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

#ifndef _ISS_LLVM__JIT_HELPER_H
#define _ISS_LLVM__JIT_HELPER_H

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
#include "jit_init.h"
#include <memory>
#include <string>
#include <tuple>
#include <unordered_map>
#include <vector>

namespace iss {

class arch_if;
class vm_if;

namespace llvm {

/**
 * get the LLVM context
 * NOTE: this is a singleton and not threadsave
 * @return the cotext
 */
::llvm::LLVMContext& getContext();

struct alignas(4 * sizeof(void*)) translation_block {
    uintptr_t f_ptr{0};
    std::array<translation_block*, 2> cont;
    ::llvm::ExecutionEngine* mod_eng{nullptr};
    explicit translation_block(uintptr_t f_ptr_, std::array<translation_block*, 2> cont_, ::llvm::ExecutionEngine* mod_eng_)
    : f_ptr(f_ptr_)
    , cont(cont_)
    , mod_eng(mod_eng_){};
    translation_block() = default;
    translation_block(translation_block const&) = default;
    translation_block(translation_block&&) = default;
    translation_block& operator=(translation_block const& other) = default;
    translation_block& operator=(translation_block&& other) = default;
    ~translation_block() { delete(mod_eng); }
};

using gen_func = std::function<::llvm::Function*(::llvm::Module*)>;

translation_block getPointerToFunction(unsigned cluster_id, uint64_t phys_addr, gen_func& generator, bool dumpEnabled);
} // namespace llvm
} // namespace iss
#endif // _ISS_LLVM__JIT_HELPER_H
