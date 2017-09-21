/*******************************************************************************
 * Copyright (C) 2017, MINRES Technologies GmbH
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

#ifndef _VM_BASE_H_
#define _VM_BASE_H_

#include "vm_if.h"
#include "debugger_if.h"
#include "debugger/target_adapter_base.h"
#include "arch_if.h"
#include "arch/traits.h"
#include "util/ities.h"
#include "util/range_lut.h"
#include "jit/MCJIThelper.h"

#include <util/logging.h>

#include <llvm/IR/Function.h>
#include <llvm/IR/Type.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/ExecutionEngine/ExecutionEngine.h>
#include "llvm/IR/MDBuilder.h"

#include <loki/AssocVector.h>
#include <vector>
#include <map>
#include <utility>
#include <sstream>
#include <chrono>

namespace iss {

template<typename ARCH>
class sim;

namespace vm {

extern llvm::cl::opt<uint32_t> LikelyBranchWeight;
extern llvm::cl::opt<uint32_t> UnlikelyBranchWeight;

enum continuation_e {CONT, BRANCH, FLUSH, TRAP};

template<typename ARCH>
class vm_base: public debugger_if , public vm_if {
public:
    typedef typename arch::traits<ARCH>::reg_e reg_e;
    typedef typename arch::traits<ARCH>::sreg_flag_e sr_flag_e;
    typedef typename arch::traits<ARCH>::virt_addr_t virt_addr_t;
    typedef typename arch::traits<ARCH>::phys_addr_t phys_addr_t;
    typedef typename arch::traits<ARCH>::addr_t addr_t;
    typedef typename arch::traits<ARCH>::code_word_t code_word_t;
    typedef typename arch::traits<ARCH>::mem_type_e mem_type_e;
    typedef typename arch::traits<ARCH>::addr_t (*func_ptr)();

    using dbg_if = iss::debugger_if;

    arch_if* get_arch() override {return &core;};

    constexpr unsigned int get_reg_width(int idx) const {
        return idx<0?arch::traits<ARCH>::NUM_REGS:arch::traits<ARCH>::reg_bit_width((reg_e)idx);
    }

    template <typename T>
    inline T get_reg(unsigned r){
        std::vector<uint8_t> res(sizeof(T),0);
        uint8_t* reg_base = core.get_regs_base_ptr() + arch::traits<ARCH>::reg_byte_offset(r);
        auto size = arch::traits<ARCH>::reg_bit_width(r)/8;
        std::copy(reg_base, reg_base+size, res.data());
        return *reinterpret_cast<T*>(&res[0]);
    }

    int start(int64_t cycles=-1) override {
        int error=0;
        if(this->debugging_enabled()) sync_exec=PRE_SYNC;
        auto start = std::chrono::high_resolution_clock::now();
        virt_addr_t pc(iss::DEBUG_FETCH, 0, get_reg<typename arch::traits<ARCH>::addr_t>(arch::traits<ARCH>::PC));
        LOG(logging::INFO)<<"Start at 0x"<<std::hex<<pc.val<<std::dec;
        try {
            vm::continuation_e cont=CONT;
            llvm::Function* func;
            func_ptr f;
            while(cycles <0 || ((int64_t)core.get_icount())<cycles) {
                try {
                    const phys_addr_t pc_p = core.v2p(pc);
                    auto it = func_map.find(pc_p.val);
                    if(it==func_map.end()){
#ifndef NDEBUG
                        LOG(logging::DEBUG)<<"Compiling and executing code for 0x"<<std::hex<<pc<<std::dec;
#endif
                        mod = jitHelper.createModule();
                        std::tie(cont, func) = disass(pc);
                        f=jitHelper.getPointerToFunction(std::move(mod), func);
                        if(cont==FLUSH)
                            func_map.clear();
                        else
                            func_map[pc_p.val]=f;
                    } else{
#ifndef NDEBUG
                        LOG(logging::DEBUG)<<"Executing code for 0x"<<std::hex<<pc<<std::dec;
#endif
                        f = it->second;
                    }
                    pc.val = f();
                } catch(trap_access& ta){
                    pc.val=core.enter_trap(ta.id, ta.addr);
                }
#ifndef NDEBUG
                LOG(logging::DEBUG)<<"continuing  @0x"<<std::hex<<pc<<std::dec;
#endif
            }
        } catch(simulation_stopped& e){
            LOG(logging::INFO)<<"ISS execution stopped with status 0x"<<std::hex<<e.state<<std::dec;
            if(e.state!=1) error=e.state;
        } catch(decoding_error& e){
            LOG(logging::ERROR)<<"ISS execution aborted at address 0x"<<std::hex<<e.addr<<std::dec;
            error=-1;
        }
        auto end = std::chrono::high_resolution_clock::now(); //end measurement here
        auto elapsed = end - start;
        auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
        LOG(logging::INFO)<<"Executed "<<core.get_icount()<<" instructions in "<<func_map.size()<<" code blocks during "<<
                millis<<"ms resulting in "<<(core.get_icount()*0.001/millis) <<"MIPS";
        return error;
    }

    void reset() { core.reset(); }

    void reset(uint64_t address) { core.reset(address); }

    void pre_instr_sync() {
        core.notify_phase(iss::arch_if::ISTART);
        if(debugging_enabled()){
            uint64_t pc=get_reg<typename arch::traits<ARCH>::addr_t>(arch::traits<ARCH>::PC);
            tgt_adapter->check_continue(pc);
        }
    }

    virtual void post_instr_sync(){
        core.notify_phase(iss::arch_if::IEND);
    }

protected:

    std::tuple<continuation_e, llvm::Function*> disass(virt_addr_t& pc) {
        unsigned blk_size=std::numeric_limits<unsigned>::max();
        unsigned cur_blk=0;
        virt_addr_t cur_pc=pc;
        processing_pc_entry addr(*this, pc, this->core.v2p(pc));
        unsigned int num_inst = 0;
        loaded_regs.clear();
        func = this->open_block_func();
        leave_blk = llvm::BasicBlock::Create(mod->getContext(), "leave", func);
        gen_leave_behavior(leave_blk);
        trap_blk = llvm::BasicBlock::Create(mod->getContext(), "trap", func);
        gen_trap_behavior(trap_blk);
        llvm::BasicBlock* bb=llvm::BasicBlock::Create(mod->getContext(), "entry", func, leave_blk);
        vm::continuation_e cont=iss::vm::CONT;
        try {
            while(cont==CONT && cur_blk<blk_size){
                builder->SetInsertPoint(bb);
                std::tie(cont, bb)=gen_single_inst_behavior(cur_pc, num_inst, bb);
                cur_blk++;
            }
            if(bb!=nullptr){
                builder->SetInsertPoint(bb);
                builder->CreateBr(leave_blk);
            }
            const phys_addr_t end_pc(this->core.v2p(--cur_pc));
            assert(this->processing_pc.top().first.val <= cur_pc.val);
            return std::make_tuple(cont, func);
        } catch(trap_access& ta){
            const phys_addr_t end_pc(this->core.v2p(--cur_pc));
            if(this->processing_pc.top().first.val <= cur_pc.val){ // close the block and return result up to here
                builder->SetInsertPoint(bb);
                builder->CreateBr(leave_blk);
                return std::make_tuple(cont, func);
            } else // re-throw if it was the first instruction
                throw ta;
        }
    }

    virtual std::tuple<continuation_e, llvm::BasicBlock*> gen_single_inst_behavior(virt_addr_t& pc_v, unsigned int& inst_cnt, llvm::BasicBlock* this_block) = 0;

    virtual void gen_trap_behavior(llvm::BasicBlock*) = 0;

    virtual void gen_leave_behavior(llvm::BasicBlock* leave_blk){
        builder->SetInsertPoint(leave_blk);
        llvm::Value * const pc_v = gen_get_reg(arch::traits<ARCH>::NEXT_PC, "next_pc");
        builder->CreateRet(pc_v);
    }

    explicit vm_base(ARCH& core, bool dump=false)
    : core(core)
    , sync_exec(NO_SYNC) // TODO: should be NO_SYNC but this needs to changes code generation
    , builder(new llvm::IRBuilder<>(getContext()))
    , jitHelper(getContext(), dump)
    , mod(nullptr)
    , func(nullptr)
    , leave_blk(nullptr)
    , trap_blk(nullptr)
    , tgt_adapter(nullptr)
    {
        //auto* const_int64_19 = llvm::ConstantInt::get(getContext(), llvm::APInt(64, this, 10));
        core_ptr = llvm::ConstantExpr::getCast(llvm::Instruction::IntToPtr,
                gen_const(64, (uintptr_t)static_cast<arch_if*>(&core)), // make sure it's the same type as in the dispatch function cause of vtable correction
                llvm::PointerType::get(get_type(8), 0));
        vm_ptr = llvm::ConstantExpr::getCast(llvm::Instruction::IntToPtr,
                gen_const(64, (uintptr_t)static_cast<vm_if*>(this)),
                llvm::PointerType::get(get_type(8), 0));
    }

    ~vm_base(){
        delete tgt_adapter;
    }

    inline llvm::Type *get_type(unsigned width) const {
        assert(width>0);
        if (width<2)
            return builder->getInt1Ty();
        else if (width<9)
            return builder->getInt8Ty();
        else if (width<17)
            return builder->getInt16Ty();
        else if(width<33)
            return builder->getInt32Ty();
        else if(width<65)
            return builder->getInt64Ty();
        assert(!"Not supported yet");
        return builder->getInt64Ty();
    }

    inline llvm::Value* adj_from64(llvm::Value* val, size_t len){
        if (len != 64)
            return builder->CreateTrunc(val, get_type(len));
        else
            return val;
    }

    inline llvm::Value *adj_to64(llvm::Value *val){
        return val->getType()->getScalarSizeInBits() == 64 ? val : builder->CreateZExt(val, builder->getInt64Ty());
    }

    inline std::string gen_var_name(const char* prefix, const char* op, int id){
        std::stringstream ss;
        ss << prefix << op << id;
        std::string str(ss.str());
        jitHelper.GenerateUniqueName(str, processing_pc.top().second.val);
        return str;
    }

    inline llvm::Value* gen_get_reg(reg_e r, const char *nm = "") {
        std::string str=gen_var_name(nm, "-reg", r);
        std::vector<llvm::Value*> args {
            core_ptr,
            reg_index(r)
        };
        return adj_from64(builder->CreateCall(mod->getFunction("get_reg"), args, str.c_str()), arch::traits<ARCH>::reg_bit_width(r));
    }

    inline void gen_set_reg(reg_e r, llvm::Value *val) {
        std::vector<llvm::Value*> args {
            core_ptr,
            reg_index(r),
            adj_to64(val)
        };
        builder->CreateCall(mod->getFunction("set_reg"), args);
    }

    inline llvm::Value* gen_get_flag(sr_flag_e flag, const char *nm = "") {
        std::string str= gen_var_name(nm, "_flag", flag);
        std::vector<llvm::Value*> args {
            core_ptr,
            llvm::ConstantInt::get(getContext(), llvm::APInt(16, flag))
        };
        llvm::Value* call = builder->CreateCall(mod->getFunction("get_flag"), args);
        return builder->CreateTrunc(call, get_type(1), str.c_str());
    }

    inline void gen_set_flag(sr_flag_e flag, llvm::Value *val) {
        std::vector<llvm::Value*> args {
            core_ptr,
            llvm::ConstantInt::get(getContext(), llvm::APInt(16, flag)),
            builder->CreateTrunc(val, get_type(1))
        };
        builder->CreateCall(mod->getFunction("set_flag"), args);
    }

    inline void gen_update_flags(iss::arch_if::operations op, llvm::Value *oper1, llvm::Value *oper2) {
        std::vector<llvm::Value*> args {
            core_ptr,
            llvm::ConstantInt::get(getContext(), llvm::APInt(16, op)),
            oper1->getType()->getScalarSizeInBits() == 64 ? oper1 : builder->CreateZExt(oper1, llvm::IntegerType::get(mod->getContext(), 64)),
                    oper2->getType()->getScalarSizeInBits() == 64 ? oper2 : builder->CreateZExt(oper2, llvm::IntegerType::get(mod->getContext(), 64))
        };
        builder->CreateCall(mod->getFunction("update_flags"), args);
    }

    inline llvm::Value* gen_read_mem(mem_type_e type, uint64_t addr, uint32_t length, const char *nm = "") {
        return gen_read_mem(type, gen_const(64, addr), length, nm);
    }

    inline llvm::Value* gen_read_mem(mem_type_e type, llvm::Value* addr, uint32_t length, const char *nm = "") {
        auto* storage = builder->CreateAlloca(llvm::IntegerType::get(mod->getContext(), length*8));
        auto* storage_ptr = builder->CreateBitCast(storage, get_type(8)->getPointerTo(0));
        std::string str= gen_var_name(nm, "_mem", type);
        std::vector<llvm::Value*> args {
            core_ptr,
            llvm::ConstantInt::get(getContext(), llvm::APInt(32, iss::VIRTUAL)),
            llvm::ConstantInt::get(getContext(), llvm::APInt(32, type)),
            adj_to64(addr),
            llvm::ConstantInt::get(getContext(), llvm::APInt(32, length)),
            storage_ptr
        };
        auto* call = builder->CreateCall(mod->getFunction("read_mem"), args);
        call->setCallingConv(llvm::CallingConv::C);
        auto* icmp = builder->CreateICmpNE(call, gen_const(8, 0UL));
        auto* label_cont = llvm::BasicBlock::Create(getContext(), "", func, this->leave_blk);
        llvm::SmallVector<uint32_t, 4> Weights(2, UnlikelyBranchWeight);
        Weights[1] = LikelyBranchWeight;
        this->builder->CreateCondBr(icmp, trap_blk, label_cont, llvm::MDBuilder(this->mod->getContext()).createBranchWeights(Weights));
        builder->SetInsertPoint(label_cont);
        switch(length){
        case 1:
        case 2:
        case 4:
        case 8:
            return builder->CreateLoad(builder->CreateBitCast(storage, get_type(length*8)->getPointerTo(0)),false);
        default:
            return storage_ptr;
        }
    }

    inline void gen_write_mem(mem_type_e type, uint64_t addr, llvm::Value *val){
        gen_write_mem(type, llvm::ConstantInt::get(getContext(), llvm::APInt(64, addr)), val);
    }

    inline void gen_write_mem(mem_type_e type, llvm::Value* addr, llvm::Value *val){
        uint32_t bitwidth = val->getType()->getIntegerBitWidth();
        auto* storage = builder->CreateAlloca(llvm::IntegerType::get(mod->getContext(), bitwidth));
        builder->CreateStore(val, storage, false);
        auto* storage_ptr = builder->CreateBitCast(storage, get_type(8)->getPointerTo(0));
        std::vector<llvm::Value*> args {
            core_ptr,
            llvm::ConstantInt::get(getContext(), llvm::APInt(32, iss::VIRTUAL)),
            llvm::ConstantInt::get(getContext(), llvm::APInt(32, type)),
            adj_to64(addr),
            llvm::ConstantInt::get(getContext(), llvm::APInt(32, bitwidth/8)),
            storage_ptr
        };
        auto* call = builder->CreateCall(mod->getFunction("write_mem"), args);
        call->setCallingConv(llvm::CallingConv::C);
        auto* icmp = builder->CreateICmpNE(call, gen_const(8, 0UL));
        auto* label_cont = llvm::BasicBlock::Create(getContext(), "", func, this->leave_blk);
        llvm::SmallVector<uint32_t, 4> Weights(2, UnlikelyBranchWeight);
        Weights[1] = LikelyBranchWeight;
        this->builder->CreateCondBr(icmp, trap_blk, label_cont, llvm::MDBuilder(this->mod->getContext()).createBranchWeights(Weights));
        builder->SetInsertPoint(label_cont);
    }

    inline
    llvm::Value* get_reg_ptr(unsigned i, unsigned size){
        void* ptr = this->core.get_regs_base_ptr()+arch::traits<ARCH>::reg_byte_offset(i);
        return llvm::ConstantExpr::getIntToPtr(
                llvm::ConstantInt::get(this->mod->getContext(), llvm::APInt(
                        8/*bits*/ * sizeof(uint8_t*),
                        reinterpret_cast<uint64_t>(ptr)
                )),
                get_type(size)
        );
    }

    template<typename T,
    typename std::enable_if< std::is_signed<T>::value>::type* = nullptr>
    inline llvm::ConstantInt * gen_const(unsigned size, T val) const{
        return llvm::ConstantInt::get(getContext(), llvm::APInt(size, val, true));
    }

    template<typename T,
    typename std::enable_if<!std::is_signed<T>::value>::type* = nullptr>
    inline llvm::ConstantInt * gen_const(unsigned size, T val) const{
        return llvm::ConstantInt::get(getContext(), llvm::APInt(size, (uint64_t)val, false));
    }

    template<typename T, typename std::enable_if<std::is_unsigned<T>::value>::type* = nullptr >
    inline llvm::Value* gen_ext(T val, unsigned size) const {
        //vm::gen_ext(this->builder, val, size, isSigned);
        return llvm::ConstantInt::get(getContext(), llvm::APInt(size, val, false));
    }

    template<typename T, typename std::enable_if<std::is_signed<T>::value>::type* = nullptr >
    inline llvm::Value* gen_ext(T val, unsigned size) const {
        //vm::gen_ext(this->builder, val, size, isSigned);
        return llvm::ConstantInt::get(getContext(), llvm::APInt(size, val, false));
    }

    template<typename T, typename std::enable_if<std::is_pointer<T>::value, int>::type* = nullptr>
    inline llvm::Value* gen_ext(T val, unsigned size) const {
        return builder->CreateZExtOrTrunc(val, builder->getIntNTy(size));
    }

    template<typename T, typename std::enable_if<std::is_integral<T>::value>::type* = nullptr>
    inline llvm::Value* gen_ext(T val, unsigned size, bool isSigned) const {
        //vm::gen_ext(this->builder, val, size, isSigned);
        return llvm::ConstantInt::get(getContext(), llvm::APInt(size, val, isSigned));
    }

    template<typename T, typename std::enable_if<std::is_pointer<T>::value, int>::type* = nullptr>
    inline llvm::Value* gen_ext(T val, unsigned size, bool isSigned) const {
        if(isSigned)
            return builder->CreateSExtOrTrunc(val, builder->getIntNTy(size));
        else
            return builder->CreateZExtOrTrunc(val, builder->getIntNTy(size));
    }

    inline llvm::Value* gen_cond_assign(llvm::Value *cond, llvm::Value *t, llvm::Value *f) const { //cond must be 1 or 0
        using namespace llvm;
        Value * const f_mask = builder->CreateSub(
                builder->CreateZExt(cond, get_type(f->getType()->getPrimitiveSizeInBits())),
                gen_const(f->getType()->getPrimitiveSizeInBits(), 1));
        Value * const t_mask = builder->CreateXor(f_mask, gen_const(f_mask->getType()->getScalarSizeInBits(), -1));
        return builder->CreateOr(builder->CreateAnd(t, t_mask), builder->CreateAnd(f, f_mask)); // (t & ~t_mask) | (f & f_mask)
    }

    inline void gen_cond_branch(llvm::Value *when, llvm::BasicBlock* then, llvm::BasicBlock* otherwise, unsigned likelyBranch=0) const { //cond must be 1 or 0
        llvm::SmallVector<uint32_t, 4> Weights(2, UnlikelyBranchWeight);
        if(likelyBranch==1) Weights[0] = LikelyBranchWeight; else if(likelyBranch==2) Weights[1] = LikelyBranchWeight;
        this->builder->CreateCondBr(when, then,    otherwise, llvm::MDBuilder(this->mod->getContext()).createBranchWeights(Weights));
    }

    inline void gen_sync(sync_type s){
        if((s & sync_exec) == PRE_SYNC)  builder->CreateCall(mod->getFunction("pre_instr_sync"), std::vector<llvm::Value*>{vm_ptr});
        if((s & sync_exec) == POST_SYNC) builder->CreateCall(mod->getFunction("post_instr_sync"),std::vector<llvm::Value*>{vm_ptr});
    }

    virtual llvm::Function* open_block_func() {
        std::string name("block");
        jitHelper.GenerateUniqueName(name, processing_pc.top().second.val);
        std::vector<llvm::Type*> mainFuncTyArgs;
        llvm::Type * ret_t = get_type(get_reg_width(arch::traits<ARCH>::PC));
        llvm::FunctionType* const mainFuncTy = llvm::FunctionType::get(ret_t, mainFuncTyArgs, false);
        llvm::Function* f = llvm::Function::Create(mainFuncTy, llvm::GlobalValue::ExternalLinkage, name.c_str(), mod.get());
        f->setCallingConv(llvm::CallingConv::C);
        return f;
    }

    llvm::ConstantInt* reg_index(unsigned r) const {
        return llvm::ConstantInt::get(getContext(), llvm::APInt(16, r));
    }
    struct processing_pc_entry{
        processing_pc_entry(vm_base<ARCH>& vm, vm_base<ARCH>::virt_addr_t pc_v, vm_base<ARCH>::phys_addr_t pc_p):vm(vm){
            vm.processing_pc.push(std::make_pair(pc_v, pc_p));
        }
        ~processing_pc_entry(){
            vm.processing_pc.pop();
        }
    protected:
        vm_base<ARCH>& vm;
    };
    ARCH& core;
    sync_type sync_exec;
    llvm::Value *core_ptr=nullptr, *vm_ptr=nullptr;
    llvm::IRBuilder<> * builder=nullptr;
    MCJIT_arch_helper<ARCH> jitHelper;
    std::unique_ptr<llvm::Module> mod;
    llvm::Function* func;
    llvm::BasicBlock *leave_blk, *trap_blk;
    //std::map<uint64_t, std::unique_ptr<vm::MCJIT_block<ARCH> > > func_map;
//    Loki::AssocVector<uint64_t, std::unique_ptr<vm::MCJIT_block<ARCH> > > func_map;
    Loki::AssocVector<uint64_t, func_ptr > func_map;

    std::stack<std::pair<virt_addr_t, phys_addr_t> > processing_pc;
    //std::pair<llvm::Value*, llvm::Value*> regs[arch::traits<ARCH>::NUM_REGS]; //latest_reg, is_dirty
    std::vector<llvm::Value*> loaded_regs{arch::traits<ARCH>::NUM_REGS, nullptr};
    iss::debugger::target_adapter_base* tgt_adapter;
};

}
}

#endif /* _VM_BASE_H_ */
