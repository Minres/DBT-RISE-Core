/*******************************************************************************
 * Copyright (C) 2023 MINRES Technologies GmbH
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
 *       alex@minres.com - initial implementation
 ******************************************************************************/

#ifndef ASMJIT_VM_BASE_H_
#define ASMJIT_VM_BASE_H_

#include <asmjit/x86/x86operand.h>
#include <cassert>
#include <cstddef>
#include <cstdlib>
#include <fmt/format.h>
#include <iss/arch/traits.h>
#include <iss/arch_if.h>
#include <iss/debugger/target_adapter_base.h>
#include <iss/debugger_if.h>
#include <iss/vm_if.h>
#include <iss/vm_plugin.h>
#include <stdexcept>
#include <util/ities.h>
#include <util/logging.h>

#include "jit_helper.h"
#include <asmjit/asmjit.h>
extern "C" {
#include <iss/vm_jit_funcs.h>
}
#include <array>
#include <chrono>
#include <iostream>
#include <map>
#include <sstream>
#include <stack>
#include <unordered_map>
#include <utility>
#include <vector>

namespace iss {

namespace asmjit {
using namespace ::asmjit;

enum continuation_e { CONT, BRANCH, FLUSH, TRAP };
enum operation { add, sub, band, bor, bxor, shl, sar, shr };
enum complex_operation { imul, mul, idiv, div, srem, urem };
enum comparison_operation { land, lor, eq, ne, lt, ltu, gt, gtu, lte, lteu, gte, gteu };
enum unary_operation { lnot, inc, dec, bnot, neg };

template <typename ARCH> class vm_base : public debugger_if, public vm_if {
    struct plugin_entry {
        sync_type sync;
        vm_plugin& plugin;
        void* plugin_ptr; // FIXME: hack
    };

public:
    using reg_e = typename arch::traits<ARCH>::reg_e;
    using sr_flag_e = typename arch::traits<ARCH>::sreg_flag_e;
    using virt_addr_t = typename arch::traits<ARCH>::virt_addr_t;
    using phys_addr_t = typename arch::traits<ARCH>::phys_addr_t;
    using addr_t = typename arch::traits<ARCH>::addr_t;
    using code_word_t = typename arch::traits<ARCH>::code_word_t;
    using mem_type_e = typename arch::traits<ARCH>::mem_type_e;
    using traits = typename arch::traits<ARCH>::traits;

    using dbg_if = iss::debugger_if;
    constexpr static unsigned blk_size = 128; // std::numeric_limits<unsigned>::max();

    arch_if* get_arch() override { return &core; };

    constexpr unsigned int get_reg_width(int idx) const {
        return idx < 0 ? arch::traits<ARCH>::NUM_REGS : arch::traits<ARCH>::reg_bit_widths[(reg_e)idx];
    }

    template <typename T> inline T get_reg(unsigned r) {
        std::vector<uint8_t> res(sizeof(T), 0);
        uint8_t* reg_base = core.get_regs_base_ptr() + arch::traits<ARCH>::reg_byte_offsets[r];
        auto size = arch::traits<ARCH>::reg_bit_widths[r] / 8;
        std::copy(reg_base, reg_base + size, res.data());
        return *reinterpret_cast<T*>(&res[0]);
    }

    using func_ptr = uint64_t (*)(uint8_t*, void*, void*);

    int start(uint64_t icount = std::numeric_limits<uint64_t>::max(), bool dump = false,
              finish_cond_e cond = finish_cond_e::COUNT_LIMIT | finish_cond_e::JUMP_TO_SELF) override {
        int error = 0;
        if(this->debugging_enabled())
            sync_exec = PRE_SYNC;
        auto start = std::chrono::high_resolution_clock::now();
        virt_addr_t pc(iss::access_type::DEBUG_FETCH, 0, get_reg<typename arch::traits<ARCH>::addr_t>(arch::traits<ARCH>::PC));
        CPPLOG(INFO) << "Start at 0x" << std::hex << pc.val << std::dec;
        try {
            continuation_e cont = CONT;

            std::function<void(jit_holder&)> generator{[this, &pc, &cont](jit_holder& jh) -> void {
                gen_block_prologue(jh);
                cont = translate(pc, jh);
                gen_block_epilogue(jh);
                // move local disass collection to global collection by appending
                this->global_disass_collection.insert(this->global_disass_collection.end(), jh.disass_collection.begin(),
                                                      jh.disass_collection.end());
                jh.disass_collection.clear();
            }};
            // explicit std::function to allow use as reference in call below
            // std::function<Function*(Module*)> gen_ref(std::ref(generator));
            translation_block *last_tb = nullptr, *cur_tb = nullptr;
            uint32_t last_branch = std::numeric_limits<uint32_t>::max();
            arch_if* const arch_if_ptr = static_cast<arch_if*>(&core);
            vm_if* const vm_if_ptr = static_cast<vm_if*>(this);
            while(!core.should_stop() && core.get_icount() < icount) {
                try {
                    // translate into physical address
                    phys_addr_t pc_p(pc.access, pc.space, pc.val);
                    if(this->core.has_mmu())
                        pc_p = this->core.virt2phys(pc);
                    // check if we have the block already compiled
                    auto it = this->func_map.find(pc_p.val);
                    if(it == this->func_map.end()) { // if not generate and compile it
                        auto res = func_map.insert(
                            std::make_pair(pc_p.val, iss::asmjit::getPointerToFunction(cluster_id, pc_p.val, generator, dump)));
                        it = res.first;
                    }
                    cur_tb = &(it->second);
                    // if we have a previous block link the just compiled one as successor of the last tb
                    if(last_tb && last_branch < 2 && last_tb->cont[last_branch] == nullptr)
                        last_tb->cont[last_branch] = cur_tb;
                    do {
                        // execute the compiled function
                        pc.val = reinterpret_cast<func_ptr>(cur_tb->f_ptr)(regs_base_ptr, arch_if_ptr, vm_if_ptr);
                        if(core.should_stop())
                            break;
                        // update last state
                        last_tb = cur_tb;
                        last_branch = core.get_last_branch();
                        auto cur_icount = core.get_icount();
                        // if the current tb has a successor assign to current tb
                        if(last_branch < 2 && cur_tb->cont[last_branch] != nullptr && cur_icount < icount)
                            cur_tb = cur_tb->cont[last_branch];
                        else // if not we need to compile one
                            cur_tb = nullptr;
                    } while(cur_tb != nullptr);
                    if(cont == FLUSH) {
                        func_map.clear();
                    }
                    if(cont == TRAP) {
                        auto it = func_map.find(pc_p.val);
                        if(it != func_map.end()) {
                            func_map.erase(it);
                        }
                    }
                } catch(trap_access& ta) {
                    pc.val = core.enter_trap(ta.id, ta.addr, 0);
                }
#ifndef NDEBUG
                CPPLOG(TRACE) << "continuing  @0x" << std::hex << pc << std::dec;
#endif
                // break; //for testing, only exec first block and end sim
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
        CPPLOG(INFO) << "Executed " << core.get_icount() << " instructions in " << func_map.size() << " code blocks during " << millis
                     << "ms resulting in " << (core.get_icount() * 0.001 / millis) << "MIPS";
        return error;
    }

    void reset() override { core.reset(); }

    void reset(uint64_t address) override { core.reset(address); }

    void pre_instr_sync() override {
        uint64_t pc = get_reg<typename arch::traits<ARCH>::addr_t>(arch::traits<ARCH>::PC);
        tgt_adapter->check_continue(pc);
    }

protected:
    continuation_e translate(virt_addr_t& pc, jit_holder& jh) {
        unsigned cur_blk = 0;
        virt_addr_t cur_pc = pc;
        phys_addr_t phys_pc(pc.access, pc.space, pc.val);
        if(this->core.has_mmu())
            phys_pc = this->core.virt2phys(pc);
        std::pair<virt_addr_t, phys_addr_t> cur_pc_mark(pc, phys_pc);
        unsigned int num_inst = 0;
        continuation_e cont = CONT;
        try {
            while(cont == CONT && cur_blk < blk_size) {
                cont = gen_single_inst_behavior(cur_pc, num_inst, jh);
                cur_blk++;
            }
            assert(cur_pc_mark.first.val <= cur_pc.val);
            return cont;
        } catch(trap_access& ta) {
            if(cur_pc_mark.first.val <= cur_pc.val) {
                return cont;
            } else
                throw ta;
        }
        return cont;
    }
    virtual continuation_e gen_single_inst_behavior(virt_addr_t&, unsigned int&, jit_holder&) = 0;
    virtual void gen_block_prologue(jit_holder&) = 0;
    virtual void gen_block_epilogue(jit_holder&) = 0;

    explicit vm_base(ARCH& core, unsigned core_id = 0, unsigned cluster_id = 0)
    : core(core)
    , core_id(core_id)
    , cluster_id(cluster_id)
    , regs_base_ptr(core.get_regs_base_ptr())
    , sync_exec(NO_SYNC)
    , tgt_adapter(nullptr) {
        sync_exec = static_cast<sync_type>(sync_exec | core.needed_sync());
    }

    ~vm_base() override {
        delete tgt_adapter;
        for(auto& each : global_disass_collection) {
            free(each);
        }
        global_disass_collection.clear();
    }

    void register_plugin(vm_plugin& plugin) override {
        if(plugin.registration("1.0", *this)) {
            plugins.push_back(plugin_entry{plugin.get_sync(), plugin, &plugin});
        }
    }
    void gen_sync(jit_holder& jh, sync_type s, unsigned inst_id) {
        if(plugins.size() /*or debugger*/)
            write_back(jh);
        for(plugin_entry e : plugins) {
            if(e.sync & s) {
                iss::instr_info_t iinfo{cluster_id, core_id, inst_id, s};
                InvokeNode* call_plugin_node;
                jh.cc.comment("//Plugin call:");
                jh.cc.invoke(&call_plugin_node, &call_plugin, FuncSignatureT<void, void*, uint64_t>());
                call_plugin_node->setArg(0, e.plugin_ptr);
                call_plugin_node->setArg(1, iinfo.backing.val);
            }
        }
        // TODO: handle Debugger
    }

    inline void gen_wait(jit_holder& jh, unsigned type) {
        InvokeNode* call_wait;
        jh.cc.comment("//gen_wait");
        jh.cc.invoke(&call_wait, &wait, FuncSignatureT<void, void*, uint64_t>());
        call_wait->setArg(0, this->get_arch());
        call_wait->setArg(1, type);
    }
    inline void gen_leave(jit_holder& jh, unsigned lvl) {
        InvokeNode* call_leave;
        jh.cc.comment("//gen_leave");
        jh.cc.invoke(&call_leave, &leave_trap, FuncSignatureT<void, void*, uint64_t>());
        call_leave->setArg(0, this->get_arch());
        call_leave->setArg(1, lvl);
        jh.next_pc = load_reg_from_mem(jh, traits::NEXT_PC);
    }

    ARCH& core;
    unsigned core_id = 0;
    unsigned cluster_id = 0;
    uint8_t* regs_base_ptr;
    sync_type sync_exec;
    std::unordered_map<uint64_t, translation_block> func_map;
    iss::debugger::target_adapter_base* tgt_adapter;
    std::vector<plugin_entry> plugins;
    std::vector<char*> global_disass_collection;

    // Asmjit generator functions for commonly used expressions
    void write_back(jit_holder& jh) {
        write_reg_to_mem(jh, jh.pc, traits::PC);
        write_reg_to_mem(jh, jh.next_pc, traits::NEXT_PC);
    }
    x86::Mem get_ptr_for(jit_holder& jh, unsigned idx) {
        switch(traits::reg_bit_widths[idx]) {
        case 8:
            return x86::ptr_8(jh.regs_base_ptr, traits::reg_byte_offsets[idx]);
        case 16:
            return x86::ptr_16(jh.regs_base_ptr, traits::reg_byte_offsets[idx]);
        case 32:
            return x86::ptr_32(jh.regs_base_ptr, traits::reg_byte_offsets[idx]);
        case 64:
            return x86::ptr_64(jh.regs_base_ptr, traits::reg_byte_offsets[idx]);
        default:
            throw std::runtime_error("Invalid reg size in get_ptr_for");
        }
    }
    x86::Gp get_reg(jit_holder& jh, unsigned size, bool is_signed = true) {
        if(is_signed)
            switch(size) {
            case 8:
                return jh.cc.newInt8();
            case 16:
                return jh.cc.newInt16();
            case 32:
                return jh.cc.newInt32();
            case 64:
                return jh.cc.newInt64();
            default:
                throw std::runtime_error("Invalid reg size in get_reg");
            }
        else
            switch(size) {
            case 8:
                return jh.cc.newUInt8();
            case 16:
                return jh.cc.newUInt16();
            case 32:
                return jh.cc.newUInt32();
            case 64:
                return jh.cc.newUInt64();
            default:
                throw std::runtime_error("Invalid reg size in get_reg");
            }
    }
    x86::Gp get_reg_for(jit_holder& jh, unsigned idx) { return get_reg(jh, traits::reg_bit_widths[idx]); }

    inline x86::Gp load_reg_from_mem(jit_holder& jh, unsigned idx) {
        auto ptr = get_ptr_for(jh, idx);
        auto reg = get_reg_for(jh, idx);
        jh.cc.mov(reg, ptr);
        return reg;
    }
    inline void write_reg_to_mem(jit_holder& jh, x86::Gp reg, unsigned idx) {
        auto ptr = get_ptr_for(jh, idx);
        jh.cc.mov(ptr, reg);
    }
    template <typename T, typename = std::enable_if_t<std::is_integral<T>::value>>
    inline x86::Gp gen_ext(jit_holder& jh, T val, unsigned size, bool is_signed) {
        auto val_reg = get_reg(jh, sizeof(val) * 8, is_signed);
        jh.cc.mov(val_reg, val);
        return gen_ext(jh, val_reg, size, is_signed);
    }
    inline x86::Gp gen_ext(jit_holder& jh, x86::Gp val, unsigned size, bool is_signed) {
        if(val.size() * 8 == size) // identity cast
            return val;
        else if(val.size() * 8 > size) // truncation
            switch(size) {
            case 8:
                return val.r8();
            case 16:
                return val.r16();
            case 32:
                return val.r32();
            default:
                throw std::runtime_error("Invalid truncation size in gen_ext");
            }
        x86::Gp ret_val = get_reg(jh, size);
        if(is_signed) {
            if(val.size() == 4 && size == 64)
                jh.cc.movsxd(ret_val, val);
            else
                jh.cc.movsx(ret_val, val);
        } else if(val.size() == 1 || val.size() == 2)
            // movzx can extend 8 and 16 bit values
            jh.cc.movzx(ret_val, val);
        else {
            assert(val.size() == 4);
            // from: http://x86asm.net/articles/x86-64-tour-of-intel-manuals/
            // Perhaps the most surprising fact is that an instruction such as MOV EAX, EBX automatically zeroes
            // upper 32 bits of RAX register
            ret_val = get_reg(jh, val.size() * 8);
            jh.cc.mov(ret_val, val);
            switch(size) {
            case 32:
                return ret_val.r32();
            case 64:
                return ret_val.r64();
            default:
                throw std::runtime_error("Invalid size in gen_ext");
            }
        }
        return ret_val;
    }

    inline x86::Gp gen_read_mem(jit_holder& jh, mem_type_e type, x86::Gp addr, uint32_t length) {
        x86::Compiler& cc = jh.cc;
        auto ret_reg = cc.newInt32();

        auto mem_type_reg = cc.newInt32();
        cc.mov(mem_type_reg, type);

        auto space_reg = cc.newInt32();
        cc.mov(space_reg, static_cast<uint16_t>(iss::address_type::VIRTUAL));

        auto val_ptr = cc.newUIntPtr();
        cc.mov(val_ptr, read_mem_buf);
        x86::Mem read_res;
        InvokeNode* invokeNode;
        x86::Gp val_reg = get_reg(jh, length * 8, true);

        switch(length) {
        case 1: {
            cc.invoke(&invokeNode, &read_mem1, FuncSignatureT<uint32_t, uint64_t, uint32_t, uint32_t, uint64_t, uintptr_t>());
            read_res = x86::ptr_8(val_ptr);
            break;
        }
        case 2: {
            cc.invoke(&invokeNode, &read_mem2, FuncSignatureT<uint32_t, uint64_t, uint32_t, uint32_t, uint64_t, uintptr_t>());
            read_res = x86::ptr_16(val_ptr);
            break;
        }
        case 4: {
            cc.invoke(&invokeNode, &read_mem4, FuncSignatureT<uint32_t, uint64_t, uint32_t, uint32_t, uint64_t, uintptr_t>());
            read_res = x86::ptr_32(val_ptr);
            break;
        }
        case 8: {
            cc.invoke(&invokeNode, &read_mem8, FuncSignatureT<uint32_t, uint64_t, uint32_t, uint32_t, uint64_t, uintptr_t>());
            read_res = x86::ptr_64(val_ptr);
            break;
        }
        default:
            throw std::runtime_error(fmt::format("Invalid length ({}) in gen_read_mem", length));
        }

        invokeNode->setRet(0, ret_reg);
        invokeNode->setArg(0, jh.arch_if_ptr);
        invokeNode->setArg(1, space_reg);
        invokeNode->setArg(2, mem_type_reg);
        invokeNode->setArg(3, addr);
        invokeNode->setArg(4, val_ptr);
        cc.cmp(ret_reg, 0);
        cc.jne(jh.trap_entry);

        cc.mov(val_reg, read_res);
        return val_reg;
    }
    inline x86::Gp gen_read_mem(jit_holder& jh, mem_type_e type, uint64_t addr, uint32_t length) {
        auto addr_reg = get_reg(jh, length * 8, true);
        jh.cc.mov(addr_reg, addr);

        return gen_read_mem(jh, type, addr_reg, length);
    }

    inline void gen_write_mem(jit_holder& jh, mem_type_e type, x86::Gp addr, x86::Gp val, uint32_t length) {
        x86::Compiler& cc = jh.cc;
        assert(val.size() == length);
        auto mem_type_reg = cc.newInt32();
        jh.cc.mov(mem_type_reg, type);
        auto space_reg = cc.newInt32();
        jh.cc.mov(space_reg, static_cast<uint16_t>(iss::address_type::VIRTUAL));
        auto ret_reg = cc.newInt32();
        InvokeNode* invokeNode;
        switch(length) {
        case 1:
            cc.invoke(&invokeNode, &write_mem1, FuncSignatureT<uint32_t, uint64_t, uint32_t, uint32_t, uint64_t, uint8_t>());
            break;
        case 2:
            cc.invoke(&invokeNode, &write_mem2, FuncSignatureT<uint32_t, uint64_t, uint32_t, uint32_t, uint64_t, uint16_t>());
            break;
        case 4:
            cc.invoke(&invokeNode, &write_mem4, FuncSignatureT<uint32_t, uint64_t, uint32_t, uint32_t, uint64_t, uint32_t>());
            break;
        case 8:
            cc.invoke(&invokeNode, &write_mem8, FuncSignatureT<uint32_t, uint64_t, uint32_t, uint32_t, uint64_t, uint64_t>());

            break;
        default:
            throw std::runtime_error("Invalid register size in gen_write_mem");
        }
        invokeNode->setRet(0, ret_reg);
        invokeNode->setArg(0, jh.arch_if_ptr);
        invokeNode->setArg(1, space_reg);
        invokeNode->setArg(2, mem_type_reg);
        invokeNode->setArg(3, addr);
        invokeNode->setArg(4, val);

        cc.cmp(ret_reg, 0);
        cc.jne(jh.trap_entry);
    }
    inline void gen_write_mem(jit_holder& jh, mem_type_e type, uint64_t addr, x86::Gp val, uint32_t length) {
        auto addr_reg = jh.cc.newUInt64();
        jh.cc.mov(addr_reg, addr);
        gen_write_mem(jh, type, addr_reg, val, length);
    }
    inline void gen_write_mem(jit_holder& jh, mem_type_e type, uint64_t addr, int64_t val, uint32_t length) {
        auto val_reg = get_reg(jh, length * 8, true);
        jh.cc.mov(val_reg, val);

        auto addr_reg = jh.cc.newUInt64();
        jh.cc.mov(addr_reg, addr);
        gen_write_mem(jh, type, addr_reg, val_reg, length);
    }
    inline void gen_write_mem(jit_holder& jh, mem_type_e type, x86::Gp addr, int64_t val, uint32_t length) {
        auto val_reg = get_reg(jh, length * 8, true);
        jh.cc.mov(val_reg, val);
        gen_write_mem(jh, type, addr, val_reg, length);
    }
    template <typename T, typename = std::enable_if_t<std::is_integral<T>::value || std::is_same<T, x86::Gp>::value>>
    x86::Gp gen_operation(jit_holder& jh, operation op, x86::Gp a, T b) {
        x86::Compiler& cc = jh.cc;
        switch(op) {
        case add: {
            // To fully comply with CoreDSL this should prepend the Carry Flag
            cc.add(a, b);
            break;
        }
        case sub: {
            cc.sub(a, b);
            break;
        }
        case band: {
            cc.and_(a, b);
            break;
        }
        case bor: {
            cc.or_(a, b);
            break;
        }
        case bxor: {
            cc.xor_(a, b);
            break;
        }
        case shl: {
            cc.shl(a, b);
            break;
        }
        case sar: {
            cc.sar(a, b);
            break;
        }
        case shr: {
            cc.shr(a, b);
            break;
        }
        default:
            throw std::runtime_error(fmt::format("Current operation {} not supported in gen_operation (operation)", op));
        }
        return a;
    }
    inline void expand_division_operand(jit_holder& jh, x86::Gp upper_half, x86::Gp dividend) {
        // This function expands the dividend into the upper half, allowing correct division of negative values
        x86::Compiler& cc = jh.cc;
        switch(dividend.size()) {
        case 2:
            cc.cwd(upper_half, dividend);
            return;
        case 4:
            cc.cdq(upper_half, dividend);
            return;
        case 8:
            cc.cqo(upper_half, dividend);
            return;
        default:
            throw std::runtime_error(fmt::format("Cannot prepare division for operand of size {}", dividend.size()));
        }
    }
    template <typename T, typename = std::enable_if_t<std::is_integral<T>::value>>
    x86::Gp gen_operation(jit_holder& jh, complex_operation op, x86::Gp a, T b) {
        x86::Gp b_reg = get_reg(jh, sizeof(b) * 8);
        jh.cc.mov(b_reg, b);
        return gen_operation(jh, op, a, b_reg);
    }
    x86::Gp gen_operation(jit_holder& jh, complex_operation op, x86::Gp a, x86::Gp b) {
        assert(a.size() == b.size());
        x86::Compiler& cc = jh.cc;
        x86::Gp overflow = get_reg(jh, a.size() * 8);
        switch(op) {
        case imul: {
            cc.imul(overflow, a, b);
            // In CoreDSL multiplication of two XLEN wide registers returns a value that is 2*XLEN wide
            assert(a.size() < 64);
            auto big_a = gen_ext(jh, a, a.size() * 8 * 2, false);
            auto big_overflow = gen_ext(jh, overflow, overflow.size() * 8 * 2, false);
            cc.shl(big_overflow, overflow.size() * 8);
            cc.or_(big_a, big_overflow);
            return big_a;
        }
        case mul: {
            assert(a.size() < 64);
            auto big_a = gen_ext(jh, a, a.size() * 8 * 2, false);
            auto big_overflow = gen_ext(jh, overflow, overflow.size() * 8 * 2, false);
            cc.shl(big_overflow, overflow.size() * 8);
            cc.or_(big_a, big_overflow);
            return big_a;
        }
        case idiv: {
            expand_division_operand(jh, overflow, a);
            cc.idiv(overflow, a, b);
            return a;
        }
        case div: {
            cc.mov(overflow, 0);
            cc.div(overflow, a, b);
            return a;
        }
        case srem: {
            // division changes the contents of the a register, in this case as a side effect. Move it out of the way
            x86::Gp a_clone = get_reg(jh, a.size() * 8);
            cc.mov(a_clone, a);
            expand_division_operand(jh, overflow, a_clone);
            cc.idiv(overflow, a_clone, b);
            return overflow;
        }
        case urem: {
            x86::Gp a_clone = get_reg(jh, a.size() * 8);
            cc.mov(a_clone, a);
            cc.mov(overflow, 0);
            cc.div(overflow, a_clone, b);
            return overflow;
        }

        default:
            throw std::runtime_error(fmt::format("Current operation {} not supported in gen_operation (complex_operation)", op));
        }
    }

    template <typename T, typename = std::enable_if_t<std::is_integral<T>::value || std::is_same<T, x86::Gp>::value>>
    x86::Gp gen_operation(jit_holder& jh, comparison_operation op, x86::Gp a, T b) {
        x86::Compiler& cc = jh.cc;
        x86::Gp tmp = cc.newInt8();
        cc.mov(tmp, 1);
        Label label_then = cc.newLabel();
        cc.cmp(a, b);
        switch(op) {
        case eq:
            cc.je(label_then);
            break;
        case ne:
            cc.jne(label_then);
            break;
        case lt:
            cc.jl(label_then);
            break;
        case ltu:
            cc.jb(label_then);
            break;
        case gt:
            cc.jg(label_then);
            break;
        case gtu:
            cc.ja(label_then);
            break;
        case lte:
            cc.jle(label_then);
            break;
        case lteu:
            cc.jbe(label_then);
            break;
        case gte:
            cc.jge(label_then);
            break;
        case gteu:
            cc.jae(label_then);
            break;
        case land: {
            Label label_false = cc.newLabel();
            cc.cmp(a, 0);
            cc.je(label_false);
            auto b_reg = cc.newInt8();
            cc.mov(b_reg, b);
            cc.cmp(b_reg, 0);
            cc.je(label_false);
            cc.jmp(label_then);
            cc.bind(label_false);
            break;
        }
        case lor: {
            cc.cmp(a, 0);
            cc.jne(label_then);
            auto b_reg = cc.newInt8();
            cc.mov(b_reg, b);
            cc.cmp(b_reg, 0);
            cc.jne(label_then);
            break;
        }
        default:
            throw std::runtime_error(fmt::format("Current operation {} not supported in gen_operation (comparison)", op));
        }
        cc.mov(tmp, 0);
        cc.bind(label_then);
        return tmp;
    }

    x86::Gp gen_operation(jit_holder& jh, unary_operation op, x86::Gp a) {
        x86::Compiler& cc = jh.cc;
        switch(op) {
        case lnot:
            throw std::runtime_error("Current operation not supported in gen_operation(lnot)");
        case inc: {
            cc.inc(a);
            break;
        }
        case dec: {
            cc.dec(a);
            break;
        }
        case bnot: {
            cc.not_(a);
            break;
        }
        case neg: {
            cc.neg(a);
            break;
        }
        default:
            throw std::runtime_error(fmt::format("Current operation {} not supported in gen_operation (unary)", op));
        }
        return a;
    }
};
} // namespace asmjit
} // namespace iss
#endif /* ASMJIT_VM_BASE_H_ */
