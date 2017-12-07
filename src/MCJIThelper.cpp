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
#include <iss/log_categories.h>
#include <llvm/Support/Debug.h> //EnableDebugBuffering
#include <llvm/Support/Error.h>
#include <llvm/Support/PrettyStackTrace.h>
#include <llvm/Support/Signals.h> // llvm::sys::PrintStackTraceOnErrorSignal()
#include <llvm/Support/TargetSelect.h>
#include <llvm/Support/raw_ostream.h> //outs()
// needed to get the execution engine linked in
#include "llvm/ExecutionEngine/SectionMemoryManager.h"
#include "llvm/IR/LegacyPassManager.h"
#include "llvm/Transforms/Scalar.h"
#include "llvm/Transforms/Scalar/GVN.h"
#include <llvm/ExecutionEngine/MCJIT.h>

#include "llvm/Analysis/Passes.h"
#include <iostream>

using namespace llvm;
using namespace iss::vm;
using namespace logging;

void iss::init_jit(int argc, char *argv[]) {
    llvm::InitializeNativeTarget();
    llvm::InitializeNativeTargetAsmPrinter();
    llvm::InitializeNativeTargetAsmParser();

    llvm::sys::PrintStackTraceOnErrorSignal(argv[0]);
    // llvm::PrettyStackTraceProgram X(argc, argv);
    // llvm::EnableDebugBuffering = true;
}

LLVMContext &iss::getContext() {
    static LLVMContext context;
    return context;
}

MCJIT_helper::MCJIT_helper(LLVMContext &context, bool dump)
: context(context)
, dumpEnabled(dump) {}

/*
std::string MCJIT_helper::GenerateUniqueName(const char *root) const {
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
*/

std::unique_ptr<legacy::FunctionPassManager>
iss::vm::MCJIT_helper::createFpm(const std::unique_ptr<llvm::Module> &mod) {
    // Create a new pass manager attached to it.
    std::unique_ptr<legacy::FunctionPassManager> fpm = llvm::make_unique<legacy::FunctionPassManager>(mod.get());
    // promote memory references to be register references
    //fpm->add(createPromoteMemoryToRegisterPass());
    // Do simple "peephole" optimizations and bit-twiddling optzns.
    //    fpm->add(createInstructionCombiningPass());
    // Reassociate expressions.
    //>    fpm->add(createReassociatePass());
    // Eliminate Common SubExpressions.
    //>    fpm->add(createGVNPass());
    // Simplify the control flow graph (deleting unreachable blocks, etc).
    //    fpm->add(createCFGSimplificationPass());
    // flatten CFG, reduce number of conditional branches by using parallel-and
    // and parallel-or mode, etc...
    //>    fpm->add(createFlattenCFGPass());
    // merges loads and stores in diamonds. Loads are hoisted into the header,
    // while stores sink into the footer.
    //fpm->add(createMergedLoadStoreMotionPass());
    // Remove redundant instructions.
    //>    fpm->add(createInstructionSimplifierPass());
    //fpm->add(createDeadStoreEliminationPass());
    // perform global value numbering and redundant load elimination
    //fpm->add(createNewGVNPass());
    // Combine loads into bigger loads.
    //fpm->add(createLoadCombinePass());

    fpm->add(createDeadStoreEliminationPass());
    // Promote allocas to registers.
    fpm->add(createPromoteMemoryToRegisterPass());
    // Do simple "peephole" optimizations and bit-twiddling optzns.
    fpm->add(createInstructionCombiningPass());
    // Reassociate expressions.
    fpm->add(createReassociatePass());
    // Eliminate Common SubExpressions.
    fpm->add(createGVNPass());
    // Simplify the control flow graph (deleting unreachable blocks, etc).
    fpm->add(createCFGSimplificationPass());

    fpm->doInitialization();
    return fpm;
}

llvm::ExecutionEngine *iss::vm::MCJIT_helper::createExecutionEngine(std::unique_ptr<llvm::Module> mod) {
    std::string ErrStr;
    EngineBuilder EEB(std::move(mod));
    //    EEB.setUseOrcMCJITReplacement(true);
    ExecutionEngine *EE = EEB.setErrorStr(&ErrStr)
                              .setMCJITMemoryManager(std::make_unique<SectionMemoryManager>())
                              .setOptLevel(CodeGenOpt::Aggressive)
                              .create();
    // Set the global so the code gen can use this.
    if (!EE) throw std::runtime_error(ErrStr.c_str());
    return EE;
}

ExecutionEngine *MCJIT_helper::compileModule(std::unique_ptr<llvm::Module> mod) {
    assert(engineMap.find(mod->getModuleIdentifier()) == engineMap.end());
    if (dumpEnabled) {
        std::error_code ec;
        std::string name(((std::string)mod->getName()) + ".il");
        llvm::raw_fd_ostream os(llvm::StringRef(name), ec, llvm::sys::fs::F_None);
        // llvm::WriteBitcodeToFile(mod, OS);
        // mod->dump();
        mod->print(os, nullptr, false, true);
        os.flush();
    }
    const std::string moduleID = mod->getModuleIdentifier();
    ExecutionEngine *ee = createExecutionEngine(std::move(mod));
    ee->finalizeObject();
    // Store this engine
    engineMap[moduleID] = ee;
    return ee;
}

uint64_t MCJIT_helper::getPointerToFunction(std::unique_ptr<Module> mod, llvm::Function *const func) {
    // Create a new pass manager attached to it.
    createFpm(mod)->run(*func);
    if (func && !func->empty()) {
//        return compileModule(std::move(mod))->getPointerToFunction(func);
        return compileModule(std::move(mod))->getFunctionAddress(func->getName());
    }
    return 0;
}

GenericValue MCJIT_helper::executeFunction(std::unique_ptr<llvm::Module> mod, const std::string &name) {
    // Look for the functions in our modules, compiling only as necessary
    Function *func = mod->getFunction(name);
    if (!func || func->empty()) throw std::runtime_error("could not find function");
    std::vector<GenericValue> args(0);
    return createExecutionEngine(std::move(mod))->runFunction(func, args);
}

