/*******************************************************************************
 * Copyright (C) 2017, 2018, MINRES Technologies GmbH
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 *    may be used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * Contributors:
 *       eyck@minres.com - initial API and implementation
 ******************************************************************************/

#ifndef LLVM_VM_BASE_H_
#define LLVM_VM_BASE_H_

#include "jit_helper.h"
#include <absl/container/flat_hash_map.h>
#include <iss/arch/traits.h>
#include <iss/arch_if.h>
#include <iss/debugger/target_adapter_base.h>
#include <iss/debugger_if.h>
#include <iss/vm_if.h>
#include <iss/vm_plugin.h>
#include <util/ities.h>
#include <util/logging.h>
#include <util/range_lut.h>

#include <llvm/ExecutionEngine/ExecutionEngine.h>
#include <llvm/IR/Function.h>
#include <llvm/IR/MDBuilder.h>
#include <llvm/IR/Type.h>

#include <array>
#include <chrono>
#include <iostream>
#include <map>
#include <sstream>
#include <stack>
#include <utility>
#include <vector>

namespace iss {

namespace llvm {
using namespace ::llvm;

enum continuation_e { CONT, BRANCH, FLUSH, TRAP, ILLEGAL_INSTR, JUMP_TO_SELF, ILLEGAL_FETCH };
enum last_branch_e { NO_JUMP = 0, KNOWN_JUMP = 1, UNKNOWN_JUMP = 2, BRANCH_TO_SELF = 3 };

void add_functions_2_module(Module* mod);

template <typename ARCH> class vm_base : public debugger_if, public vm_if {
    struct plugin_entry {
        sync_type sync;
        vm_plugin& plugin;
        Value* plugin_ptr;
    };

public:
    using reg_e = typename arch::traits<ARCH>::reg_e;
    using reg_t = typename arch::traits<ARCH>::reg_t;
    using sr_flag_e = typename arch::traits<ARCH>::sreg_flag_e;
    using virt_addr_t = typename arch::traits<ARCH>::virt_addr_t;
    using phys_addr_t = typename arch::traits<ARCH>::phys_addr_t;
    using addr_t = typename arch::traits<ARCH>::addr_t;
    using code_word_t = typename arch::traits<ARCH>::code_word_t;
    using mem_type_e = typename arch::traits<ARCH>::mem_type_e;

    using dbg_if = iss::debugger_if;
    using translation_block = iss::llvm::translation_block;

    constexpr static unsigned blk_size = 128; // std::numeric_limits<unsigned>::max();

    arch_if* get_arch() override { return &core; };

    constexpr unsigned int get_reg_width(int idx) const {
        return idx < 0 ? arch::traits<ARCH>::NUM_REGS : arch::traits<ARCH>::reg_bit_widths[(reg_e)idx];
    }

    template <typename T> inline T get_reg_val(unsigned r) {
        std::vector<uint8_t> res(sizeof(T), 0);
        uint8_t* reg_base = regs_base_ptr + arch::traits<ARCH>::reg_byte_offsets[r];
        auto size = arch::traits<ARCH>::reg_bit_widths[r] / 8;
        std::copy(reg_base, reg_base + size, res.data());
        return *reinterpret_cast<T*>(&res[0]);
    }
    template <typename T = reg_t> inline T& get_reg(unsigned r) {
        return *reinterpret_cast<T*>(regs_base_ptr + arch::traits<ARCH>::reg_byte_offsets[r]);
    }

    using func_ptr = uint64_t (*)(uint8_t*, void*, void*);

    int start(uint64_t icount_limit = std::numeric_limits<uint64_t>::max(), bool dump = false,
              finish_cond_e cond = finish_cond_e::ICOUNT_LIMIT | finish_cond_e::JUMP_TO_SELF) override {
        int error = 0;
        uint32_t was_illegal = 0;
        if(this->debugging_enabled())
            sync_exec |= PRE_SYNC;
        auto start = std::chrono::high_resolution_clock::now();
        virt_addr_t pc(iss::access_type::DEBUG_FETCH, 0, get_reg<typename arch::traits<ARCH>::addr_t>(arch::traits<ARCH>::PC));
        CPPLOG(INFO) << "Start at 0x" << std::hex << pc.val << std::dec;
        try {
            continuation_e cont = CONT;
            // struct to minimize the type size of the closure below to allow SSO
            struct {
                vm_base* vm;
                virt_addr_t& pc;
                continuation_e& cont;
            } param = {this, pc, cont};
            std::function<Function*(Module*)> generator{[&param, icount_limit](Module* m) -> Function* {
                Function* func;
                param.vm->mod = m;
                param.vm->setup_module(m);
                std::tie(param.cont, func) = param.vm->translate(param.pc, icount_limit);
                param.vm->mod = nullptr;
                param.vm->func = nullptr;
                return func;
            }};
            // explicit std::function to allow use as reference in call below
            // std::function<Function*(Module*)> gen_ref(std::ref(generator));
            translation_block *last_tb = nullptr, *cur_tb = nullptr;
            auto& last_branch = get_reg(reg_e::LAST_BRANCH);
            uint64_t& cur_icount = get_reg<uint64_t>(reg_e::ICOUNT);
            arch_if* const arch_if_ptr = static_cast<arch_if*>(&core);
            vm_if* const vm_if_ptr = static_cast<vm_if*>(this);
            while(!core.should_stop() && cur_icount < icount_limit) {
                try {
                    // translate into physical address
                    phys_addr_t pc_p(pc.access, pc.space, pc.val);
                    // check if we have the block already compiled
                    auto it = this->func_map.find(pc_p.val);
                    if(it == this->func_map.end()) { // if not generate and compile it
                        auto res = func_map.insert(
                            std::make_pair(pc_p.val, iss::llvm::getPointerToFunction(cluster_id, pc_p.val, generator, dump)));
                        it = res.first;
                    }
                    cur_tb = &(it->second);
                    if(cont == JUMP_TO_SELF) {
                        // Execute the block we just compiled, but we know it will be the last one
                        reinterpret_cast<func_ptr>(cur_tb->f_ptr)(regs_base_ptr, arch_if_ptr, vm_if_ptr);
                        throw simulation_stopped(0);
                    }
                    // if we have a previous block link the just compiled one as successor of the last tb
                    if(last_tb && last_branch < 2 && last_tb->cont[last_branch] == nullptr)
                        last_tb->cont[last_branch] = cur_tb;
                    do {
                        // execute the compiled function
                        pc.val = reinterpret_cast<func_ptr>(cur_tb->f_ptr)(regs_base_ptr, arch_if_ptr, vm_if_ptr);
                        if(core.should_stop() || last_branch == BRANCH_TO_SELF)
                            throw simulation_stopped(0);
                        // update last state
                        last_tb = cur_tb;
                        // if the current tb has a successor assign to current tb
                        if(last_branch < 2 && cur_tb->cont[last_branch] != nullptr && cur_icount < icount_limit) {
                            cur_tb = cur_tb->cont[last_branch];
                            // update cont, as it only gets set when a new fptr gets created
                            cont = static_cast<continuation_e>(last_branch);
                        } else // if not we need to compile one
                            cur_tb = nullptr;
                    } while(cur_tb != nullptr);
                    if(cont == FLUSH)
                        func_map.clear();
                    if(cont == ILLEGAL_INSTR) {
                        if(was_illegal > 2) {
                            CPPLOG(ERR) << "ISS execution aborted after trying to execute illegal instructions 3 times in a row";
                            error = -1;
                            break;
                        }
                        was_illegal++;
                    } else {
                        was_illegal = 0;
                    }
                } catch(trap_access& ta) {
                    pc.val = core.enter_trap(1 << 16, ta.addr, 0);
                }
#ifndef NDEBUG
                CPPLOG(TRACE) << "continuing  @0x" << std::hex << pc << std::dec;
#endif
            }
        } catch(simulation_stopped& e) {
            CPPLOG(INFO) << "ISS execution stopped with status 0x" << std::hex << e.state << std::dec;
            if(e.state != 1)
                error = e.state;
        } catch(decoding_error& e) {
            CPPLOG(ERR) << "ISS execution aborted at address 0x" << std::hex << e.addr << std::dec;
            error = -1;
        }
        auto end = std::chrono::high_resolution_clock::now(); // end measurement
        // here
        auto elapsed = end - start;
        auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
        uint64_t& cur_icount = get_reg<uint64_t>(reg_e::ICOUNT);
        CPPLOG(INFO) << "Executed " << cur_icount << " instructions in " << func_map.size() << " code blocks during " << millis
                     << "ms resulting in " << (cur_icount * 0.001 / millis) << "MIPS";
        return error;
    }

    void reset() override { core.reset(); }

    void reset(uint64_t address) override { core.reset(address); }

    void pre_instr_sync() override {
        uint64_t pc = get_reg<typename arch::traits<ARCH>::addr_t>(arch::traits<ARCH>::PC);
        tgt_adapter->check_continue(pc);
    }

protected:
    std::tuple<continuation_e, Function*> translate(virt_addr_t pc, uint64_t icount_limit) {
        unsigned cur_blk_size = 0;
        // loaded_regs.clear();
        func = this->open_block_func(pc);
        leave_blk = BasicBlock::Create(mod->getContext(), "leave", func);
        gen_leave_behavior(leave_blk);
        BasicBlock* bb = BasicBlock::Create(mod->getContext(), "entry", func, leave_blk);
        builder.SetInsertPoint(bb);
        // this needs to happen before calling gen_trap_behavior
        tval = new llvm::GlobalVariable(*mod, llvm::Type::getInt64Ty(mod->getContext()), false, llvm::GlobalValue::ExternalLinkage,
                                        llvm::ConstantInt::get(llvm::Type::getInt64Ty(mod->getContext()), 0), "tval");
        trap_blk = BasicBlock::Create(mod->getContext(), "trap", func);
        gen_trap_behavior(trap_blk);
        continuation_e cont = CONT;
        while(cont == CONT && cur_blk_size < blk_size && cur_blk_size < icount_limit) {
            builder.SetInsertPoint(bb);
            std::tie(cont, bb) = gen_single_inst_behavior(pc, bb);
            cur_blk_size++;
        }
        if(bb != nullptr) {
            builder.SetInsertPoint(bb);
            builder.CreateBr(leave_blk);
        }
        if(cont == ILLEGAL_FETCH && cur_blk_size == 1) {
            throw trap_access(0, pc.val);
        }
        return std::make_tuple(cont, func);
    }

    void GenerateUniqueName(std::string& str, uint64_t mod) const {
        std::array<char, 21> buf;
        ::snprintf(buf.data(), buf.size(), "@0x%016lX_", mod);
        str += buf.data();
    }

    virtual void setup_module(Module* m) { add_functions_2_module(m); }

    virtual std::tuple<continuation_e, BasicBlock*> gen_single_inst_behavior(virt_addr_t& pc_v, BasicBlock* this_block) = 0;

    virtual void gen_trap_behavior(BasicBlock*) = 0;

    virtual void gen_leave_behavior(BasicBlock* leave_blk) {
        builder.SetInsertPoint(leave_blk);
        Value* const pc_v = gen_get_reg(arch::traits<ARCH>::NEXT_PC);
        builder.CreateRet(pc_v);
    }

    explicit vm_base(ARCH& core, unsigned core_id = 0, unsigned cluster_id = 0)
    : core(core)
    , core_id(core_id)
    , cluster_id(cluster_id)
    , regs_base_ptr(core.get_regs_base_ptr())
    , builder(::iss::llvm::getContext()) {
        sync_exec = static_cast<sync_type>(sync_exec | core.needed_sync());
    }
    explicit vm_base(std::unique_ptr<ARCH> unique_core_ptr, unsigned core_id = 0, unsigned cluster_id = 0)
    : core(*unique_core_ptr)
    , owning_core_ptr(std::move(unique_core_ptr))
    , core_id(core_id)
    , cluster_id(cluster_id)
    , regs_base_ptr(core.get_regs_base_ptr())
    , builder(::iss::llvm::getContext()) {
        sync_exec = static_cast<sync_type>(sync_exec | core.needed_sync());
    }

    ~vm_base() override { delete tgt_adapter; }

    void register_plugin(vm_plugin& plugin) override {
        if(plugin.registration("1.0", *this)) {
            // This is wrong, needs ptrType
            auto* plugin_addr = ConstantInt::get(::iss::llvm::getContext(), APInt(64, (uint64_t)&plugin));
            Value* ptr = ConstantExpr::getIntToPtr(plugin_addr, PointerType::getUnqual(Type::getInt8Ty(::iss::llvm::getContext())));
            plugins.push_back(plugin_entry{plugin.get_sync(), plugin, ptr});
        }
    }

    inline Value* adj_to64(Value* val) {
        return val->getType()->getScalarSizeInBits() == 64 ? val : builder.CreateZExt(val, builder.getInt64Ty());
    }
    /*
   Getter Functions to allow easy usage of LLVM API
   */
    inline Type* get_type(unsigned width) {
        assert(width > 0);
        if(width < 2)
            return builder.getInt1Ty();
        else if(width < 9)
            return builder.getInt8Ty();
        else if(width < 17)
            return builder.getInt16Ty();
        else if(width < 33)
            return builder.getInt32Ty();
        else if(width < 65)
            return builder.getInt64Ty();
        assert(!"Not supported yet");
        return builder.getInt64Ty();
    }
    inline Type* get_typeptr(unsigned i) { return get_type(arch::traits<ARCH>::reg_bit_widths[i]); }

    inline Value* get_reg_ptr(unsigned i) { return vm_base<ARCH>::get_reg_ptr(i, arch::traits<ARCH>::reg_bit_widths[i]); }

    inline Value* get_reg_ptr(unsigned i, unsigned size) {
        auto x = builder.CreateAdd(this->builder.CreatePtrToInt(regs_ptr, get_type(64)),
                                   this->gen_const(64U, arch::traits<ARCH>::reg_byte_offsets[i]), "reg_offs_ptr");
        return this->builder.CreateIntToPtr(x, get_type(size)->getPointerTo(0));
    }
    /*
    Generator functions to create often used IR
    */
    inline Value* gen_get_reg(reg_e r) {
        std::vector<Value*> args{core_ptr, ConstantInt::get(::iss::llvm::getContext(), APInt(16, r))};
        auto reg_size = arch::traits<ARCH>::reg_bit_widths[r];
        auto ret = builder.CreateCall(mod->getFunction("get_reg"), args);
        return reg_size == 64 ? ret : builder.CreateTrunc(ret, get_type(reg_size));
    }

    inline void gen_set_reg(reg_e r, Value* val) {
        std::vector<Value*> args{core_ptr, ConstantInt::get(::iss::llvm::getContext(), APInt(16, r)), adj_to64(val)};
        builder.CreateCall(mod->getFunction("set_reg"), args);
    }

    inline Value* gen_get_flag(sr_flag_e flag, const char* nm = "") {
        std::vector<Value*> args{core_ptr, ConstantInt::get(::iss::llvm::getContext(), APInt(16, flag))};
        Value* call = builder.CreateCall(mod->getFunction("get_flag"), args);
        return builder.CreateTrunc(call, get_type(1));
    }

    inline void gen_set_flag(sr_flag_e flag, Value* val) {
        std::vector<Value*> args{core_ptr, ConstantInt::get(::iss::llvm::getContext(), APInt(16, flag)),
                                 builder.CreateTrunc(val, get_type(1))};
        builder.CreateCall(mod->getFunction("set_flag"), args);
    }

    inline void gen_update_flags(iss::arch_if::operations op, Value* oper1, Value* oper2) {
        std::vector<Value*> args{
            core_ptr, ConstantInt::get(::iss::llvm::getContext(), APInt(16, op)),
            oper1->getType()->getScalarSizeInBits() == 64 ? oper1 : builder.CreateZExt(oper1, IntegerType::get(mod->getContext(), 64)),
            oper2->getType()->getScalarSizeInBits() == 64 ? oper2 : builder.CreateZExt(oper2, IntegerType::get(mod->getContext(), 64))};
        builder.CreateCall(mod->getFunction("update_flags"), args);
    }

    inline Value* gen_read_mem(mem_type_e type, uint64_t addr, uint32_t length, const char* nm = "") {
        return gen_read_mem(type, gen_const(64, addr), length, nm);
    }

    inline Value* gen_read_mem(mem_type_e type, Value* addr, uint32_t length, const char* nm = "") {
        auto* storage = builder.CreateAlloca(IntegerType::get(mod->getContext(), length * 8));
        auto* storage_ptr = builder.CreateBitCast(storage, get_type(8)->getPointerTo(0));
        std::vector<Value*> args{core_ptr,
                                 ConstantInt::get(::iss::llvm::getContext(), APInt(32, static_cast<uint16_t>(iss::address_type::VIRTUAL))),
                                 ConstantInt::get(::iss::llvm::getContext(), APInt(32, type)),
                                 adj_to64(addr),
                                 ConstantInt::get(::iss::llvm::getContext(), APInt(32, length)),
                                 storage_ptr};
        auto* call = builder.CreateCall(mod->getFunction("read_mem"), args);
        call->setCallingConv(CallingConv::C);
        auto* icmp = builder.CreateICmpNE(call, gen_const(8, 0UL));
        auto* label_cont = BasicBlock::Create(::iss::llvm::getContext(), "", func, this->leave_blk);
        this->builder.CreateCondBr(icmp, trap_blk, label_cont, MDBuilder(this->mod->getContext()).createBranchWeights(4, 64));
        builder.SetInsertPoint(label_cont);
        switch(length) {
        case 1:
        case 2:
        case 4:
        case 8:
            return builder.CreateLoad(this->get_type(length * 8), builder.CreateBitCast(storage, get_type(length * 8)->getPointerTo(0)),
                                      false);
        default:
            throw std::runtime_error("Invalid read attempt");
        }
    }
    template <typename T>
    inline typename std::enable_if<std::is_integral<T>::value>::type gen_write_mem(mem_type_e type, T addr, Value* val) {
        gen_write_mem(type, ConstantInt::get(::iss::llvm::getContext(), APInt(sizeof(T) * 8, static_cast<uint64_t>(addr))), val);
    }

    inline void gen_write_mem(mem_type_e type, Value* addr, Value* val) {
        uint32_t bitwidth = val->getType()->getIntegerBitWidth();
        auto* storage = builder.CreateAlloca(IntegerType::get(mod->getContext(), bitwidth));
        builder.CreateStore(val, storage, false);
        auto* storage_ptr = builder.CreateBitCast(storage, get_type(8)->getPointerTo(0));
        std::vector<Value*> args{core_ptr,
                                 ConstantInt::get(::iss::llvm::getContext(), APInt(32, static_cast<uint16_t>(iss::address_type::VIRTUAL))),
                                 ConstantInt::get(::iss::llvm::getContext(), APInt(32, type)),
                                 adj_to64(addr),
                                 ConstantInt::get(::iss::llvm::getContext(), APInt(32, bitwidth / 8)),
                                 storage_ptr};
        auto* call = builder.CreateCall(mod->getFunction("write_mem"), args);
        call->setCallingConv(CallingConv::C);
        auto* icmp = builder.CreateICmpNE(call, gen_const(8, 0UL));
        auto* label_cont = BasicBlock::Create(::iss::llvm::getContext(), "", func, this->leave_blk);
        this->builder.CreateCondBr(icmp, trap_blk, label_cont, MDBuilder(this->mod->getContext()).createBranchWeights(4, 64));
        builder.SetInsertPoint(label_cont);
    }

    template <typename T, typename std::enable_if<std::is_signed<T>::value>::type* = nullptr>
    inline ConstantInt* gen_const(unsigned size, T val) const {
        return ConstantInt::get(::iss::llvm::getContext(), APInt(size, val, true));
    }

    template <typename T, typename std::enable_if<!std::is_signed<T>::value>::type* = nullptr>
    inline ConstantInt* gen_const(unsigned size, T val) const {
        return ConstantInt::get(::iss::llvm::getContext(), APInt(size, (uint64_t)val, false));
    }

    template <typename V, typename W, typename = std::enable_if_t<std::is_integral<V>::value && std::is_integral<W>::value>>
    inline Value* gen_slice(Value* val, V bit, W width) {
        auto res_width = 8;
        if(width > 8)
            res_width = 16;
        if(width > 16)
            res_width = 32;
        if(width > 32)
            res_width = 64;
        // analog to bit_sub in scc util
        // T res = (v >> bit) & ((T(1) << width) - 1);
        auto val_size = val->getType()->getPrimitiveSizeInBits();
        Value* rhs = this->gen_const(val_size, (1ULL << width) - 1);
        if(!bit)
            return gen_ext(builder.CreateAnd(val, rhs), res_width);
        else
            return gen_ext(builder.CreateAnd(builder.CreateLShr(val, gen_const(val_size, bit)), rhs), res_width);
    }

    template <typename T, typename std::enable_if<std::is_unsigned<T>::value>::type* = nullptr>
    inline Value* gen_ext(T val, unsigned size) const {
        return ConstantInt::get(::iss::llvm::getContext(), APInt(size, val, false));
    }

    template <typename T, typename std::enable_if<std::is_signed<T>::value>::type* = nullptr>
    inline Value* gen_ext(T val, unsigned size) const {
        return ConstantInt::get(::iss::llvm::getContext(), APInt(size, val, false));
    }

    template <typename T, typename std::enable_if<std::is_pointer<T>::value, int>::type* = nullptr>
    inline Value* gen_ext(T val, unsigned size) {
        return builder.CreateZExtOrTrunc(val, builder.getIntNTy(size));
    }

    template <typename T, typename std::enable_if<std::is_integral<T>::value>::type* = nullptr>
    inline Value* gen_ext(T val, unsigned size, bool isSigned) const {
        return ConstantInt::get(::iss::llvm::getContext(), APInt(size, val, isSigned));
    }
    inline Value* gen_bool(Value* val) {
        if(val->getType()->isIntegerTy(1)) {
            return val;
        }
        Constant* zero = ConstantInt::get(val->getType(), 0);
        return builder.CreateICmpNE(val, zero, "tobool");
    }

    template <typename T, typename std::enable_if<std::is_pointer<T>::value, int>::type* = nullptr>
    inline Value* gen_ext(T val, unsigned size, bool isSigned) {
        if(isSigned)
            return builder.CreateSExtOrTrunc(val, builder.getIntNTy(size));
        else
            return builder.CreateZExtOrTrunc(val, builder.getIntNTy(size));
    }

    inline Value* gen_cond_assign(Value* cond, Value* t, Value* f) { // cond must be 1 or 0
        using namespace llvm;
        auto f_size = f->getType()->getPrimitiveSizeInBits();
        Value* const f_mask = builder.CreateSub(builder.CreateZExt(cond, get_type(f_size)), gen_const(f_size, 1));
        Value* const t_mask = builder.CreateXor(f_mask, gen_const(f_mask->getType()->getScalarSizeInBits(), -1));
        return builder.CreateOr(builder.CreateAnd(t, t_mask), builder.CreateAnd(f, f_mask)); // (t & ~t_mask) | (f & f_mask)
    }

    inline void gen_cond_branch(Value* cond, BasicBlock* then, BasicBlock* otherwise,
                                bool likelyTrueBranch = 0) { // cond must be 1 or 0
        if(likelyTrueBranch)
            this->builder.CreateCondBr(cond, then, otherwise, MDBuilder(this->mod->getContext()).createBranchWeights(64, 4));
        else
            this->builder.CreateCondBr(cond, then, otherwise, MDBuilder(this->mod->getContext()).createBranchWeights(4, 64));
    }

    // NO_SYNC = 0, PRE_SYNC = 1, POST_SYNC = 2, ALL_SYNC = 3
    const std::array<const iss::arch_if::exec_phase, 4> notifier_mapping = {
        {iss::arch_if::ISTART, iss::arch_if::ISTART, iss::arch_if::IEND, iss::arch_if::ISTART}};

    inline void gen_sync(sync_type s, unsigned inst_id) {
        if(s == PRE_SYNC) {
            if(debugging_enabled())
                builder.CreateCall(mod->getFunction("pre_instr_sync"), std::vector<Value*>{vm_ptr});
        }
        if((s & sync_exec))
            builder.CreateCall(mod->getFunction("notify_phase"), std::vector<Value*>{core_ptr, gen_const(32, notifier_mapping[s])});
        iss::instr_info_t iinfo{cluster_id, core_id, inst_id, s};
        for(plugin_entry e : plugins) {
            if(e.sync & s) {
                builder.CreateCall(mod->getFunction("call_plugin"), std::vector<Value*>{
                                                                        e.plugin_ptr,
                                                                        gen_const(64, iinfo.backing.val),
                                                                    });
            }
        }
    }

    virtual Function* open_block_func(phys_addr_t pc) {
        std::string name("block");
        GenerateUniqueName(name, pc.val);
        std::vector<Type*> mainFuncTyArgs{Type::getInt8Ty(mod->getContext()), Type::getInt8Ty(mod->getContext()),
                                          Type::getInt8Ty(mod->getContext())};
        Type* ret_t = get_type(get_reg_width(arch::traits<ARCH>::PC));
        FunctionType* const mainFuncTy = FunctionType::get(ret_t, mainFuncTyArgs, false);
        Function* f = Function::Create(mainFuncTy, GlobalValue::ExternalLinkage, name.c_str(), mod);
        f->setCallingConv(CallingConv::C);
        auto iter = f->arg_begin();
        regs_ptr = dyn_cast<Value>(iter++);
        core_ptr = dyn_cast<Value>(iter++);
        vm_ptr = dyn_cast<Value>(iter++);
        regs_ptr->setName("regs_ptr");
        core_ptr->setName("core_ptr");
        vm_ptr->setName("vm_ptr");
        return f;
    }

    ARCH& core;
    std::unique_ptr<ARCH> owning_core_ptr;
    unsigned core_id = 0;
    unsigned cluster_id = 0;
    uint8_t* regs_base_ptr;
    sync_type sync_exec{sync_type::NO_SYNC};
    absl::flat_hash_map<uint64_t, translation_block> func_map;
    IRBuilder<> builder{iss::llvm::getContext()};
    // non-owning pointers
    Module* mod{nullptr};
    Function* func{nullptr};
    Value *core_ptr = nullptr, *vm_ptr = nullptr, *regs_ptr = nullptr;
    BasicBlock* leave_blk{nullptr};
    BasicBlock* trap_blk{nullptr};
    // std::vector<Value *> loaded_regs{arch::traits<ARCH>::NUM_REGS, nullptr};
    iss::debugger::target_adapter_base* tgt_adapter{nullptr};
    std::vector<plugin_entry> plugins;
    GlobalVariable* tval;
};
} // namespace llvm
} // namespace iss
#endif /* LLVM_VM_BASE_H_ */
