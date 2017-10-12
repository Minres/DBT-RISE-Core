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

#ifndef _MCJITHELPER_H_
#define _MCJITHELPER_H_

#include "llvm/ExecutionEngine/GenericValue.h"
#include "llvm/IR/LegacyPassManager.h"
#include <iostream>
#include <iss/arch/traits.h>
#include <llvm/ExecutionEngine/ExecutionEngine.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/Error.h>
#include <sstream>

#include <unordered_map>
#include <vector>

namespace iss {
/**
 * get the LLVM context
 * NOTE: this is a singleton and not threadsave
 * @return the cotext
 */
llvm::LLVMContext &getContext();
/**
 * initialize the LLVM infrastructure
 * @param argc the number of CLI arguments
 * @param argv the array of CLI arguments
 */
void init_jit(int argc, char *argv[]);

namespace vm {
/**
 * MCJIT helper class
 */
class MCJIT_helper {
public:
    /**
     * constructor
     * @param context the LLVM context to use
     * @param dump dump the generated IR per module
     */
    MCJIT_helper(llvm::LLVMContext &context, bool dump = false);
    /**
     * generate a unique name from a character array using internal static counter
     * @param root
     * @return the generated name
     */
    std::string GenerateUniqueName(const char *root) const;
    /**
     * Generate a unique name based on str by appending mod in hex notation
     * @param str the base string
     * @param mod the modification count
     */
    void GenerateUniqueName(std::string &str, uint64_t count) const;
    /**
     * create a module with all global function declarations
     * @return the module pointer
     */
    std::unique_ptr<llvm::Module> createModule();
    /**
     * compile the function named <name> of module mod
     * @param mod the module to be compiled
     * @param name the name of the toplevel entry function
     */
    void *getPointerToFunction(std::unique_ptr<llvm::Module> mod, const std::string &name) {
        // Look for the function in our modules
        llvm::Function *func = mod->getFunction(name);
        return getPointerToFunction(std::move(mod), name);
    }
    /**
     * compile the function named <name> of module mod
     * @param mod the module to be compiled
     * @param func the toplevel entry function
     */
    void *getPointerToFunction(std::unique_ptr<llvm::Module> mod, llvm::Function *const func);
    /**
     * Execute the specified function with the specified arguments, and return the
     * result.
     * @param mod the module to be compiled
     * @param name the toplevel entry function
     * @return the result
     */
    llvm::GenericValue executeFunction(std::unique_ptr<llvm::Module> mod, const std::string &name);

protected:
    llvm::ExecutionEngine *createExecutionEngine(std::unique_ptr<llvm::Module> mod);
    llvm::ExecutionEngine *compileModule(std::unique_ptr<llvm::Module> mod);

    void add_functions_2_module(llvm::Module *mod);

    std::unique_ptr<llvm::legacy::FunctionPassManager> createFpm(const std::unique_ptr<llvm::Module> &mod);

private:
    llvm::LLVMContext &context;
    std::unordered_map<std::string, llvm::ExecutionEngine *> engineMap;
    const bool dumpEnabled;
};
/**
 * template wrapper to get get rid of casting in code
 */
template <typename ARCH> class MCJIT_arch_helper : public MCJIT_helper {
public:
    using fPtr_t = typename arch::traits<ARCH>::addr_t (*)();
    /**
     * constructor
     * @param context the LLVM context
     * @param dump dump the generated IR per module
     */
    MCJIT_arch_helper(llvm::LLVMContext &context, bool dump = false)
    : MCJIT_helper(context, dump) {}

    /**
     * wrapper to get correctly typed function pointer from compilation
     * @param m the module to compile
     * @param f the toplevel entry function of the module
     * @return the function pointer
     */
    fPtr_t getPointerToFunction(std::unique_ptr<llvm::Module> mod, llvm::Function *const func) {
        return (fPtr_t)MCJIT_helper::getPointerToFunction(std::move(mod), func);
    }

    typename arch::traits<ARCH>::addr_t executeFunction(std::unique_ptr<llvm::Module> mod, const llvm::Function *func) {
        return MCJIT_helper::executeFunction(std::move(mod), func->getName()).IntVal.getZExtValue();
    }
};
}
}
#endif /* _MCJITHELPER_H_ */
