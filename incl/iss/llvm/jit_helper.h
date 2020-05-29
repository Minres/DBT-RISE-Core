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

#ifndef _MCJITHELPER_H_
#define _MCJITHELPER_H_

#include "llvm/ExecutionEngine/GenericValue.h"
#include "llvm/IR/LegacyPassManager.h"
#include <iostream>
#include <iss/arch/traits.h>
#include <llvm/ExecutionEngine/ExecutionEngine.h>
#include <llvm/IR/IRBuilder.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/Error.h>
#include <sstream>

#include "boost/variant.hpp"
#include <memory>
#include <tuple>
#include <unordered_map>
#include <vector>
#include <string>

namespace iss {
/**
 * initialize the LLVM infrastructure
 */
void init_jit();

/**
 * initialize the LLVM infrastructure including stack trace pretty printer
 * @param argc the number of CLI arguments
 * @param argv the array of CLI arguments
 */
void init_jit_debug(int argc, const char * const argv[]);

class arch_if;
class vm_if;

namespace llvm {

/**
 * get the LLVM context
 * NOTE: this is a singleton and not threadsave
 * @return the cotext
 */
::llvm::LLVMContext &getContext();

struct alignas(4 * sizeof(void *)) translation_block {
    uintptr_t f_ptr = 0;
    std::array<translation_block *, 2> cont;
    ::llvm::ExecutionEngine *mod_eng;
    explicit translation_block(uintptr_t f_ptr_, std::array<translation_block *, 2> cont_,
                               ::llvm::ExecutionEngine *mod_eng_)
    : f_ptr(f_ptr_)
    , cont(cont_)
    , mod_eng(mod_eng_) {}
};

using gen_func = std::function<::llvm::Function *(::llvm::Module *)>;

translation_block getPointerToFunction(unsigned cluster_id, uint64_t phys_addr, gen_func &generator, bool dumpEnabled);
}
}
#endif /* _MCJITHELPER_H_ */
