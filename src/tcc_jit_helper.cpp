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

using namespace llvm;
using namespace logging;

namespace iss {

namespace tcc {

translation_block getPointerToFunction(unsigned cluster_id, uint64_t phys_addr, gen_func &generator, bool dumpEnabled) {
#ifndef NDEBUG
    LOG(TRACE) << "Compiling and executing code for 0x" << std::hex << phys_addr << std::dec;
#endif
    static unsigned i = 0;
    auto code = generator();
    if (dumpEnabled) {
        std::string name(fmt::format("tcc_jit_#%X.c", ++i));
        std::ofstream os(name);
        os<<code<<std::endl;
    }
    auto tcc = tcc_new(); assert(tcc);
    tcc_set_output_type(tcc, TCC_OUTPUT_MEMORY);
//    tcc_add_symbol(tcc, "add", add);
//    tcc_add_symbol(tcc, "hello", hello);
    /* relocate the code */
    assert(tcc_relocate(tcc, TCC_RELOCATE_AUTO) >= 0);
    /* get entry symbol */
    auto func = tcc_get_symbol(tcc, "foo");
    tcc_delete(tcc);
    return translation_block(reinterpret_cast<uintptr_t>(func), {nullptr, nullptr});
}
}
}

