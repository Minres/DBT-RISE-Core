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
 * Contributors:
 *       eyck@minres.com - initial API and implementation
 ******************************************************************************/

#include "jit_helper.h"
#include <iss/log_categories.h>
#include <llvm/Support/Debug.h> //EnableDebugBuffering
#include <llvm/Support/Error.h>
#include <llvm/Support/TargetSelect.h>
#include <llvm/Support/PrettyStackTrace.h>
#include <llvm/Support/Signals.h> // llvm::sys::PrintStackTraceOnErrorSignal()
#include <llvm/Support/TargetSelect.h>
#include <llvm/Support/raw_ostream.h> //outs()
// needed to get the execution engine linked in
#include <llvm/ExecutionEngine/MCJIT.h>

#include "llvm/Support/CodeGen.h"

#include <array>
#include <iostream>
#include <memory>

#include "llvm/IR/Module.h"
#include <llvm/ExecutionEngine/ExecutionEngine.h>
#include <llvm/IR/DataLayout.h>
#include <llvm/IR/LLVMContext.h>
#include <llvm/Support/MemoryBuffer.h>

using namespace llvm;
using namespace logging;

namespace iss {

void init_jit() {
    InitializeNativeTarget();
    InitializeNativeTargetAsmPrinter();
    InitializeNativeTargetAsmParser();
}

void init_jit_debug(int argc, const char* const argv[]) {
    init_jit();
#ifdef LLVM_DEBUG
    sys::PrintStackTraceOnErrorSignal(argv[0]);
    PrettyStackTraceProgram X(argc, argv);
    EnableDebugBuffering = true;
#endif
}

namespace llvm {

LLVMContext& getContext() {
    static LLVMContext context;
    return context;
}

translation_block getPointerToFunction(unsigned cluster_id, uint64_t phys_addr, std::function<Function*(Module*)>& generator,
                                       bool dumpEnabled) {
#ifndef NDEBUG
    CPPLOG(TRACE) << "Compiling and executing code for 0x" << std::hex << phys_addr << std::dec;
#endif
    static unsigned i = 0;
    std::array<char, 32> s;
    sprintf(s.data(), "llvm_jit_%u", ++i);
    auto mod = std::make_unique<Module>(s.data(), getContext());
    auto* f = generator(mod.get());
    assert(f != nullptr && "Generator function did return nullptr");
    if(dumpEnabled) {
        std::error_code ec;
        std::string name(((std::string)mod->getName()) + ".ll");
        raw_fd_ostream os(StringRef(name), ec); // sys::fs::F_None);
        // WriteBitcodeToFile(mod, OS);
        // mod->dump();
        mod->print(os, nullptr, false, true);
        os.flush();
    }
    //mod->setTargetTriple(sys::getProcessTriple());
    std::string ErrStr;
    EngineBuilder eb(std::move(mod)); // eb and ee take ownership of module
    TargetOptions to;
    to.EnableFastISel = true;
    to.GuaranteedTailCallOpt = false;
    ExecutionEngine* ee =
        eb.setEngineKind(EngineKind::JIT).setTargetOptions(to).setErrorStr(&ErrStr).setOptLevel(CodeGenOptLevel::None).create();
    if(!ee)
        throw std::runtime_error(ErrStr);
    ee->setVerifyModules(false);
    return std::move(translation_block(ee->getFunctionAddress(f->getName().str()), {nullptr, nullptr}, ee));
}
} // namespace llvm
} // namespace iss
