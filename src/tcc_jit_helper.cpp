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

#include <iss/tcc/jit_helper.h>
#include <iss/log_categories.h>
#include <fmt/format.h>
#include <array>
#include <iostream>
#include <memory>
#include <fstream>
#include <stdexcept>

using namespace logging;

namespace iss {

namespace tcc {

std::ostream& write_prologue(std::ostream& os){
    os<<"#define bool    _Bool\n";
    os<<"#define true    1\n";
    os<<"#define false   0\n";
    os<<"typedef __SIZE_TYPE__ size_t;\n";
    os<<"typedef __PTRDIFF_TYPE__ ssize_t;\n";
    os<<"typedef __WCHAR_TYPE__ wchar_t;\n";
    os<<"typedef __PTRDIFF_TYPE__ ptrdiff_t;\n";
    os<<"typedef __PTRDIFF_TYPE__ intptr_t;\n";
    os<<"typedef __SIZE_TYPE__ uintptr_t;\n";
    os<<"#define __int8_t_defined\n";
    os<<"typedef signed char int8_t;\n";
    os<<"typedef signed short int int16_t;\n";
    os<<"typedef signed int int32_t;\n";
    os<<"#ifdef __LP64__\n";
    os<<"typedef signed long int int64_t;\n";
    os<<"#else\n";
    os<<"typedef signed long long int int64_t;\n";
    os<<"#endif\n";
    os<<"typedef unsigned char uint8_t;\n";
    os<<"typedef unsigned short int uint16_t;\n";
    os<<"typedef unsigned int uint32_t;\n";
    os<<"#ifdef __LP64__\n";
    os<<"typedef unsigned long int uint64_t;\n";
    os<<"#else\n";
    os<<"typedef unsigned long long int uint64_t;\n";
    os<<"#endif\n";
    os<<"#define NULL ((void*)0)\n";
    os<<"#define offsetof(type, field) ((size_t)&((type *)0)->field)\n";
    os<<"void *alloca(size_t size);\n";
    os<<"#if defined (__need_wint_t)\n";
    os<<"#ifndef _WINT_T\n";
    os<<"#define _WINT_T\n";
    os<<"typedef __WINT_TYPE__ wint_t;\n";
    os<<"#endif\n";
    os<<"#undef __need_wint_t\n";
    os<<"#endif\n";

    os<<"extern int read_mem1(void*, uint32_t, uint32_t, uint64_t, uint8_t*);\n";
    os<<"extern int write_mem1(void*, uint32_t, uint32_t, uint64_t, uint8_t);\n";
    os<<"extern int read_mem2(void*, uint32_t, uint32_t, uint64_t, uint16_t*);\n";
    os<<"extern int write_mem2(void*, uint32_t, uint32_t, uint64_t, uint16_t);\n";
    os<<"extern int read_mem4(void*, uint32_t, uint32_t, uint64_t, uint32_t*);\n";
    os<<"extern int write_mem4(void*, uint32_t, uint32_t, uint64_t, uint32_t);\n";
    os<<"extern int read_mem8(void*, uint32_t, uint32_t, uint64_t, uint64_t*);\n";
    os<<"extern int write_mem8(void*, uint32_t, uint32_t, uint64_t, uint64_t);\n";
    os<<"extern uint64_t enter_trap(void*, uint64_t, uint64_t, uint64_t);\n";
    os<<"extern uint64_t leave_trap(void*, uint64_t);\n";
    os<<"extern void wait(void*, uint64_t);\n";
    os<<"extern void print_string(void*, char*);\n";
    os<<"extern void print_disass(void*, uint64_t, char*);\n";
    os<<"extern void pre_instr_sync(void*);\n";
    os<<"extern void notify_phase(void*, uint32_t);\n";
    os<<"extern void call_plugin(void*, uint64_t) ;\n";
    os<<"extern uint32_t fget_flags();\n";
    os<<"extern uint32_t fadd_s(uint32_t v1, uint32_t v2, uint8_t mode);\n";
    os<<"extern uint32_t fsub_s(uint32_t v1, uint32_t v2, uint8_t mode);\n";
    os<<"extern uint32_t fmul_s(uint32_t v1, uint32_t v2, uint8_t mode);\n";
    os<<"extern uint32_t fdiv_s(uint32_t v1, uint32_t v2, uint8_t mode);\n";
    os<<"extern uint32_t fsqrt_s(uint32_t v1, uint8_t mode);\n";
    os<<"extern uint32_t fcmp_s(uint32_t v1, uint32_t v2, uint32_t op) ;\n";
    os<<"extern uint32_t fcvt_s(uint32_t v1, uint32_t op, uint8_t mode);\n";
    os<<"extern uint32_t fmadd_s(uint32_t v1, uint32_t v2, uint32_t v3, uint32_t op, uint8_t mode);\n";
    os<<"extern uint32_t fsel_s(uint32_t v1, uint32_t v2, uint32_t op);\n";
    os<<"extern uint32_t fclass_s( uint32_t v1 );\n";
    os<<"extern uint32_t fconv_d2f(uint64_t v1, uint8_t mode);\n";
    os<<"extern uint64_t fconv_f2d(uint32_t v1, uint8_t mode);\n";
    os<<"extern uint64_t fadd_d(uint64_t v1, uint64_t v2, uint8_t mode);\n";
    os<<"extern uint64_t fsub_d(uint64_t v1, uint64_t v2, uint8_t mode);\n";
    os<<"extern uint64_t fmul_d(uint64_t v1, uint64_t v2, uint8_t mode);\n";
    os<<"extern uint64_t fdiv_d(uint64_t v1, uint64_t v2, uint8_t mode);\n";
    os<<"extern uint64_t fsqrt_d(uint64_t v1, uint8_t mode);\n";
    os<<"extern uint64_t fcmp_d(uint64_t v1, uint64_t v2, uint32_t op);\n";
    os<<"extern uint64_t fcvt_d(uint64_t v1, uint32_t op, uint8_t mode);\n";
    os<<"extern uint64_t fmadd_d(uint64_t v1, uint64_t v2, uint64_t v3, uint32_t op, uint8_t mode);\n";
    os<<"extern uint64_t fsel_d(uint64_t v1, uint64_t v2, uint32_t op) ;\n";
    os<<"extern uint64_t fclass_d(uint64_t v1  );\n";
    os<<"extern uint64_t fcvt_32_64(uint32_t v1, uint32_t op, uint8_t mode);\n";
    os<<"extern uint32_t fcvt_64_32(uint64_t v1, uint32_t op, uint8_t mode);\n";
    os<<"extern uint32_t unbox_s(uint64_t v);\n";

    return os;
}

translation_block getPointerToFunction(unsigned cluster_id, uint64_t phys_addr, gen_func &generator, bool dumpEnabled) {
#ifndef NDEBUG
    LOG(TRACE) << "Compiling and executing code for 0x" << std::hex << phys_addr << std::dec;
#endif
    static unsigned i = 0;
    auto res = generator();
    if (dumpEnabled) {
        std::string name(fmt::format("tcc_jit_{}.c", ++i));
        std::ofstream ofs(name);
        ofs<<std::get<1>(res)<<std::endl;
    }
    auto tcc = tcc_new();
    if(!tcc) throw std::runtime_error("could not create TCC instance");
    tcc_set_output_type(tcc, TCC_OUTPUT_MEMORY);
    tcc_set_options(tcc, "-fno-common");
    tcc_set_options(tcc, "-w");
//    tcc_set_options(tcc, "-g");
    /* relocate the code */
    auto result=tcc_compile_string(tcc, std::get<1>(res).c_str());
    if(result) throw std::runtime_error("could not compile translated code");
    int size = tcc_relocate(tcc, nullptr);
    if(!size) throw std::runtime_error("TCC did not return reasonable code size");
    auto* fmem = malloc(size);
    result=tcc_relocate(tcc, fmem);
    if(result) throw std::runtime_error("could not relocate compiled code");
    /* get entry symbol */
    auto func = tcc_get_symbol(tcc, std::get<0>(res).c_str());
    tcc_delete(tcc);
    return translation_block(func, {nullptr, nullptr}, fmem);
}
}
}

