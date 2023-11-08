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
 * Contributors:
 *       alex@minres.com - initial implementation
 ******************************************************************************/

#include <iss/log_categories.h>

#include "jit_helper.h"
#include <array>
#include <exception>
#include <fmt/format.h>
#include <iostream>
#include <memory>
#include <sys/mman.h>

using namespace asmjit;
using namespace logging;
namespace iss {
namespace asmjit {
// according to https://github.com/LuaDist/tcc/blob/255ba0e8e34f999ee840407ce4e9c444dfd312bf/libtcc.c#L400
int set_pages_executable(void* ptr, unsigned long length) {
#ifdef _WIN32
    unsigned long old_protect;
    return VirtualProtect(ptr, length, PAGE_EXECUTE_READWRITE, &old_protect);
#else
    unsigned long start, end;
    start = (unsigned long)ptr & ~(PAGESIZE - 1);
    end = (unsigned long)ptr + length;
    end = (end + PAGESIZE - 1) & ~(PAGESIZE - 1);
    return mprotect((void*)start, end - start, PROT_READ | PROT_WRITE | PROT_EXEC);
#endif
};
class MyErrorHandler : public ErrorHandler {
public:
    void handleError(Error err, const char* message, BaseEmitter* origin) override {
        throw std::runtime_error(fmt::format("AsmJit error: {}", message));
    }
};

translation_block getPointerToFunction(unsigned cluster_id, uint64_t phys_addr, std::function<void(jit_holder&)>& generator,
                                       bool dumpEnabled) {
#ifndef NDEBUG
    LOG(TRACE) << "Compiling and executing code for 0x" << std::hex << phys_addr << std::dec;
#endif
    static int i = 0;
    JitRuntime rt;
    FileLogger logger(nullptr);
    MyErrorHandler myErrorHandler;
    CodeHolder code;
    code.init(rt.environment(), rt.cpuFeatures());
    if(dumpEnabled) {
        std::string name(fmt::format("asmjit_{}.asm", ++i));
        FILE* log_file = fopen(name.c_str(), "w");
        logger.setFile(log_file);
        code.setLogger(&logger);
    }
    code.setErrorHandler(&myErrorHandler);
    x86::Compiler cc(&code);
    FuncNode* funcNode = cc.addFunc(FuncSignatureT<uint64_t, uint8_t*, void*, void*>());
    x86::Gp regs_base_ptr = cc.newUIntPtr("regs_base_ptr");
    x86::Gp arch_if_ptr = cc.newIntPtr("arch_if_ptr");
    x86::Gp vm_if_ptr = cc.newIntPtr("vm_if_ptr");
    funcNode->setArg(0, regs_base_ptr);
    funcNode->setArg(1, arch_if_ptr);
    funcNode->setArg(2, vm_if_ptr);
    Label trap_entry = cc.newNamedLabel("\ntrap_entry");
    jit_holder jh{cc, regs_base_ptr, arch_if_ptr, vm_if_ptr, trap_entry};
    generator(jh);

    cc.endFunc();
    cc.finalize();
    code.flatten();
    // allocate memory and make it executable
    auto size = code.codeSize();
    auto* fmem = malloc(size);
    int err = code.relocateToBase(uintptr_t(fmem));
    if(err)
        throw std::runtime_error("could not relocate compiled code");
    err = code.copyFlattenedData(fmem, size);
    if(err)
        throw std::runtime_error("could not copy compiled code");
    err = set_pages_executable(fmem, size);
    if(err)
        throw std::runtime_error("could not set memory as executable");
    // abort();
    return translation_block(fmem, {nullptr, nullptr}, fmem);
}
} // namespace asmjit
} // namespace iss
