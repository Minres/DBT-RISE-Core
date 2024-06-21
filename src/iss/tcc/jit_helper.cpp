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
#include <array>
#include <fmt/format.h>
#include <fstream>
#include <iostream>
#include <iss/log_categories.h>
#include <iss/vm_jit_funcs.h>
#include <memory>
#include <stdexcept>

using namespace logging;

namespace iss {
namespace tcc {

translation_block getPointerToFunction(unsigned cluster_id, uint64_t phys_addr, gen_func& generator, bool dumpEnabled) {
#ifndef NDEBUG
    CPPLOG(TRACE) << "Compiling and executing code for 0x" << std::hex << phys_addr << std::dec;
#endif
    static unsigned i = 0;
    auto res = generator();
    if(dumpEnabled) {
        std::string name(fmt::format("tcc_jit_{}.c", ++i));
        std::ofstream ofs(name);
        ofs << std::get<1>(res) << std::endl;
    }
    auto tcc = tcc_new();
    if(!tcc)
        throw std::runtime_error("could not create TCC instance");
    tcc_set_output_type(tcc, TCC_OUTPUT_MEMORY);
    tcc_set_options(tcc, "-fno-common");
    tcc_set_options(tcc, "-w");
    /* relocate the code */
    auto result = tcc_compile_string(tcc, std::get<1>(res).c_str());
    if(result)
        throw std::runtime_error("could not compile translated code");
    int size = tcc_relocate(tcc, nullptr);
    if(!size)
        throw std::runtime_error("TCC did not return reasonable code size");
    auto* fmem = malloc(size);
    result = tcc_relocate(tcc, fmem);
    if(result)
        throw std::runtime_error("could not relocate compiled code");
    /* get entry symbol */
    auto func = tcc_get_symbol(tcc, std::get<0>(res).c_str());
    tcc_delete(tcc);
    return translation_block(func, {nullptr, nullptr}, fmem);
}
} // namespace tcc
} // namespace iss
