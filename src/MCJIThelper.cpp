////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2017, MINRES Technologies GmbH
// All rights reserved.
// 
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
// 
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
// 
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
// 
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
// 
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
// 
// Contributors:
//       eyck@minres.com - initial API and implementation
////////////////////////////////////////////////////////////////////////////////

#include <iss/jit/MCJIThelper.h>
#include <util/logging.h>
#include <llvm/Support/Error.h>
#include <llvm/Support/Debug.h> //EnableDebugBuffering
#include <llvm/Support/raw_ostream.h> //outs()
#include <llvm/Support/TargetSelect.h>
#include <llvm/Support/PrettyStackTrace.h>
#include <llvm/Support/Signals.h>// llvm::sys::PrintStackTraceOnErrorSignal()
// needed to get the execution engine linked in
#include <llvm/ExecutionEngine/MCJIT.h>
#include "llvm/ExecutionEngine/SectionMemoryManager.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/Transforms/Scalar.h"
#include "llvm/Transforms/Scalar/GVN.h"

#include "llvm/Analysis/Passes.h"
#include <iostream>

using namespace llvm;
using namespace iss::vm;
using namespace logging;

void iss::init_jit(int argc, char *argv[]){
    llvm::InitializeNativeTarget();
    llvm::InitializeNativeTargetAsmPrinter();
    llvm::InitializeNativeTargetAsmParser();

    llvm::sys::PrintStackTraceOnErrorSignal(argv[0]);
    //llvm::PrettyStackTraceProgram X(argc, argv);
    //llvm::EnableDebugBuffering = true;
}

LLVMContext& iss::getContext() {
    static LLVMContext context;
    return context;
}

MCJIT_helper::MCJIT_helper(LLVMContext& context, bool dump)
        : context(context), dumpEnabled(dump) {
}

MCJIT_helper::~MCJIT_helper() {
}

std::string MCJIT_helper::GenerateUniqueName(const char *root) const{
    static int i = 0;
    char s[20];
    sprintf(s, "%s#%X_", root, i++);
    return std::string(s);
}

void MCJIT_helper::GenerateUniqueName(std::string &str, uint64_t mod) const {
    char buf[21];
    ::snprintf(buf, sizeof(buf), "@0x%016lX_", mod);
    str += buf;
}

std::unique_ptr<llvm::Module> MCJIT_helper::createModule() {
    //  create a new Module.
    auto mod_name = GenerateUniqueName("mcjit_module_");
    auto mod = std::make_unique<Module>(mod_name, context);
    add_functions_2_module(mod.get());
    return mod;
}

std::unique_ptr<legacy::FunctionPassManager> iss::vm::MCJIT_helper::createFpm(const std::unique_ptr<llvm::Module>& mod) {
    // Create a new pass manager attached to it.
    std::unique_ptr<legacy::FunctionPassManager> fpm = llvm::make_unique<legacy::FunctionPassManager>(mod.get());
    // promote memory references to be register references
    fpm->add(createPromoteMemoryToRegisterPass());
    // Do simple "peephole" optimizations and bit-twiddling optzns.
//    fpm->add(createInstructionCombiningPass());
    // Reassociate expressions.
//>    fpm->add(createReassociatePass());
    // Eliminate Common SubExpressions.
//>    fpm->add(createGVNPass());
    // Simplify the control flow graph (deleting unreachable blocks, etc).
//    fpm->add(createCFGSimplificationPass());
    // flatten CFG, reduce number of conditional branches by using parallel-and and parallel-or mode, etc...
//>    fpm->add(createFlattenCFGPass());
    // merges loads and stores in diamonds. Loads are hoisted into the header, while stores sink into the footer.
    fpm->add(createMergedLoadStoreMotionPass());
    // Remove redundant instructions.
//>    fpm->add(createInstructionSimplifierPass());
    // Combine loads into bigger loads.
    fpm->add(createLoadCombinePass());
    fpm->doInitialization();
    return fpm;
}

llvm::ExecutionEngine* iss::vm::MCJIT_helper::createExecutionEngine(std::unique_ptr<llvm::Module> mod) {
    std::string ErrStr;
    EngineBuilder EEB(std::move(mod));
//    EEB.setUseOrcMCJITReplacement(true);
    ExecutionEngine *EE = EEB
                .setErrorStr(&ErrStr)
                .setMCJITMemoryManager(std::make_unique<SectionMemoryManager>())
                .setOptLevel(CodeGenOpt::Aggressive)
                .create();
    // Set the global so the code gen can use this.
    if (!EE)  throw std::runtime_error(ErrStr.c_str());
    return EE;
}

ExecutionEngine *MCJIT_helper::compileModule(std::unique_ptr<llvm::Module> mod) {
    assert(engineMap.find(mod->getModuleIdentifier()) == engineMap.end());
    if(dumpEnabled){
        std::error_code ec;
        std::string name(((std::string)mod->getName())+".il");
        llvm::raw_fd_ostream os(llvm::StringRef(name), ec, llvm::sys::fs::F_None);
        // llvm::WriteBitcodeToFile(mod, OS);
        // mod->dump();
        mod->print(os, nullptr, false, true);
        os.flush();
    }

    const std::string moduleID = mod->getModuleIdentifier();
    ExecutionEngine* ee = createExecutionEngine(std::move(mod));
    ee->finalizeObject();
    // Store this engine
    engineMap[moduleID] = ee;
    return ee;
}

void *MCJIT_helper::getPointerToFunction(std::unique_ptr<Module> mod, llvm::Function* const func) {
    // Create a new pass manager attached to it.
    //createFpm(mod)->run(*func);
    if (func && !func->empty()) {
        return compileModule(std::move(mod))->getPointerToFunction(func);
    }
    return NULL;
}

GenericValue MCJIT_helper::executeFunction(std::unique_ptr<llvm::Module> mod, const std::string &name){
    // Look for the functions in our modules, compiling only as necessary
    Function *func = mod->getFunction(name);
    if (!func || func->empty()) throw std::runtime_error("could not find function");
    std::vector<GenericValue> args(0);
    return createExecutionEngine(std::move(mod))->runFunction(func, args);
}

#define INT_TYPE(L) IntegerType::get(mod->getContext(), L)
#define VOID_TYPE Type::getVoidTy(mod->getContext())
#define THIS_PTR_TYPE INT_TYPE(8)->getPointerTo()
#define FDECLL(NAME,RET, ...) \
Function* NAME ## _func = CurrentModule->getFunction(#NAME);\
if (!NAME ## _func) {\
    std::vector<Type*> NAME ## _args {__VA_ARGS__};\
    FunctionType* NAME ## _type = FunctionType::get(RET, NAME ## _args, false);\
    NAME ## _func = Function::Create(NAME ## _type, GlobalValue::ExternalLinkage, #NAME, CurrentModule);\
    NAME ## _func ->setCallingConv(CallingConv::C);\
}

#define FDECL(NAME,RET, ...) \
        std::vector<Type*> NAME ## _args {__VA_ARGS__};\
        FunctionType* NAME ## _type = llvm::FunctionType::get(RET, NAME ## _args, false);\
        mod->getOrInsertFunction(#NAME, NAME ## _type);

                using namespace llvm;

void MCJIT_helper::add_functions_2_module(Module* mod){
    //Type* voidType = Type::getVoidTy(CurrentModule->getContext());
    FDECL(get_reg,         INT_TYPE(64), THIS_PTR_TYPE, INT_TYPE(16));
    FDECL(set_reg,         VOID_TYPE,    THIS_PTR_TYPE, INT_TYPE(16), INT_TYPE(64));
    FDECL(get_flag,        INT_TYPE(1),  THIS_PTR_TYPE, INT_TYPE(16));
    FDECL(set_flag,        VOID_TYPE,    THIS_PTR_TYPE, INT_TYPE(16), INT_TYPE(1));
    FDECL(update_flags,    VOID_TYPE,    THIS_PTR_TYPE, INT_TYPE(16), INT_TYPE(64), INT_TYPE(64));
    FDECL(fetch,           INT_TYPE(8),  THIS_PTR_TYPE, INT_TYPE(32), INT_TYPE(32), INT_TYPE(64), INT_TYPE(32), INT_TYPE(8)->getPointerTo());
    FDECL(fetch_dbg,       INT_TYPE(8),  THIS_PTR_TYPE, INT_TYPE(32), INT_TYPE(32), INT_TYPE(64), INT_TYPE(32), INT_TYPE(8)->getPointerTo());
    FDECL(read_mem,        INT_TYPE(8),  THIS_PTR_TYPE, INT_TYPE(32), INT_TYPE(32), INT_TYPE(64), INT_TYPE(32), INT_TYPE(8)->getPointerTo());
    FDECL(write_mem,       INT_TYPE(8),  THIS_PTR_TYPE, INT_TYPE(32), INT_TYPE(32), INT_TYPE(64), INT_TYPE(32), INT_TYPE(8)->getPointerTo());
    FDECL(read_mem_dbg,    INT_TYPE(8),  THIS_PTR_TYPE, INT_TYPE(32), INT_TYPE(32), INT_TYPE(64), INT_TYPE(32), INT_TYPE(8)->getPointerTo());
    FDECL(write_mem_dbg,   INT_TYPE(8),  THIS_PTR_TYPE, INT_TYPE(32), INT_TYPE(32), INT_TYPE(64), INT_TYPE(32), INT_TYPE(8)->getPointerTo());
    FDECL(enter_trap,      INT_TYPE(64), THIS_PTR_TYPE, INT_TYPE(64), INT_TYPE(64));
    FDECL(leave_trap,      INT_TYPE(64), THIS_PTR_TYPE, INT_TYPE(64));
    FDECL(wait,            VOID_TYPE,    THIS_PTR_TYPE, INT_TYPE(64));
    FDECL(print_string,    VOID_TYPE,    THIS_PTR_TYPE, INT_TYPE(8)->getPointerTo());
    FDECL(print_disass,    VOID_TYPE,    THIS_PTR_TYPE, INT_TYPE(8)->getPointerTo());
    FDECL(pre_instr_sync,  VOID_TYPE,    THIS_PTR_TYPE);
    FDECL(post_instr_sync, VOID_TYPE,    THIS_PTR_TYPE);
}

#include <iss/iss.h>
#include <iss/arch_if.h>
#include <util/logging.h>

typedef uint8_t* this_t;
// Use default logger
//static el::Logger* get_logger(){
//    static el::Logger* logger = el::Loggers::getLogger("disass", true);
//    return logger;
//}

extern "C" {
uint64_t get_reg(this_t iface, int16_t idx) {
    std::vector<uint8_t>data(8, 0);
    ((iss::arch_if*)iface)->get_reg(idx, data);
#ifdef EXEC_LOGGING
    LOG(TRACE)<<"EXEC: read reg "<<idx<<" of core "<<iface<<" getting value 0x"<<hex<<res<<dec;
#endif
    return *(reinterpret_cast<uint64_t*>(&data[0]));
}

void set_reg(this_t iface, int16_t idx, uint64_t value) {
#ifdef EXEC_LOGGING
    LOG(TRACE)<<"EXEC: write reg "<<idx<<" of core "<<iface<<" with value 0x"<<hex<<value<<dec;
#endif
    std::vector<uint8_t>data(8, 0);
    *(reinterpret_cast<uint64_t*>(&data[0]))=value;
    ((iss::arch_if*)iface)->set_reg(idx, data);
}

bool get_flag(this_t iface, int16_t flag) {
    bool res = ((iss::arch_if*)iface)->get_flag(flag);
#ifdef EXEC_LOGGING
    LOG(TRACE)<<"EXEC: read flag "<<flag<<" of core "<<iface<<" getting value 0x"<<hex<<res<<dec;
#endif
    return res;
}

void set_flag(this_t iface, int16_t flag, bool value) {
#ifdef EXEC_LOGGING
    LOG(TRACE)<<"EXEC: write flag "<<flag<<" of core "<<iface<<" with value 0x"<<value;
#endif
    ((iss::arch_if*)iface)->set_flag(flag, value);
}

void update_flags(this_t iface, int16_t op, uint64_t opr1, uint64_t opr2){
    ((iss::arch_if*)iface)->update_flags((iss::arch_if::operations)op, opr1, opr2);
}

uint8_t fetch(this_t iface, uint32_t addr_type, uint32_t space, uint64_t addr, uint32_t length, uint8_t* data){
    return ((iss::arch_if*)iface)->read(iss::addr_t{iss::FETCH|addr_type, space, addr}, length, data);
}

uint8_t fetch_dbg(this_t iface, uint32_t addr_type, uint32_t space, uint64_t addr, uint32_t length, uint8_t* data){
    return ((iss::arch_if*)iface)->read(iss::addr_t{iss::DEBUG_FETCH|addr_type, space, addr}, length, data);
}
uint8_t read_mem(this_t iface, uint32_t addr_type, uint32_t space, uint64_t addr, uint32_t length, uint8_t* data){
    return ((iss::arch_if*)iface)->read(iss::addr_t{iss::READ|addr_type, space, addr}, length, data);
}

uint8_t write_mem(this_t iface, uint32_t addr_type, uint32_t space, uint64_t addr, uint32_t length, uint8_t* data){
#ifdef EXEC_LOGGING
    LOG(TRACE)<<"EXEC: write mem "<<(unsigned)type<<" of core "<<iface<<" at addr 0x"<<hex<<addr<<" with value 0x"<<data<<dec<<" of len "<<length;
#endif
    return ((iss::arch_if*)iface)->write(iss::addr_t{iss::WRITE|addr_type, space, addr}, length, data);
}

uint8_t read_mem_dbg(this_t iface, uint32_t addr_type, uint32_t space, uint64_t addr, uint32_t length, uint8_t* data){
    return ((iss::arch_if*)iface)->read(iss::addr_t{iss::DEBUG_READ|addr_type, space, addr}, length, data);
}

uint8_t write_mem_dbg(this_t iface, uint32_t addr_type, uint32_t space, uint64_t addr, uint32_t length, uint8_t* data){
    return ((iss::arch_if*)iface)->write(iss::addr_t{iss::DEBUG_WRITE|addr_type, space, addr}, length, data);
}

uint64_t enter_trap(this_t iface, uint64_t flags, uint64_t addr){
    return ((iss::arch_if*)iface)->enter_trap(flags, addr);
}

uint64_t leave_trap(this_t iface, uint64_t flags){
    return ((iss::arch_if*)iface)->leave_trap(flags);
}

void wait(this_t iface, uint64_t flags){
    ((iss::arch_if*)iface)->wait_until(flags);
}

void print_string(this_t iface, char* str){
    LOG(DEBUG)<<"[EXEC] "<<str;
}

void print_disass(this_t iface, char* str){
    LOG(INFO)<<str<<((iss::arch_if*)iface)->get_additional_disass_info();
}

void pre_instr_sync(this_t iface){
    iss::vm_if* vm = reinterpret_cast<iss::vm_if*>(iface);
    vm->pre_instr_sync();
}

void post_instr_sync(this_t iface){
    ((iss::vm_if*)iface)->post_instr_sync();
}
}


