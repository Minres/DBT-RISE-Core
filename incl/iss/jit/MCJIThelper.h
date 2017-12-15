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
#include <llvm/IR/IRBuilder.h>
#include <llvm/Support/Error.h>
#include <sstream>

#include <unordered_map>
#include <loki/AssocVector.h>
#include <vector>
#include "boost/variant.hpp"

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
    struct MCJIT_block {
        boost::variant<llvm::ExecutionEngine*, llvm::Module*> mod_eng{(llvm::ExecutionEngine*)nullptr};
        uint64_t f_ptr=0;
    };
public:
    /**
     * constructor
     * @param context the LLVM context to use
     * @param dump dump the generated IR per module
     */
    MCJIT_helper(llvm::LLVMContext &context = getContext());
    /**
     * compile the function named <name> of module mod
     *
     * @param cluster_id    id of the cluster the code is JIT'ed for
     * @param phys_addr     physical address of the JIT#ed code
     * @param generator     function generating JIT'ing the target code
     * @param dump_enabled  dump IR in text form if enabled
     * @return the pointer to the compiled function
     */
    template <typename FTYPE>
    inline
    FTYPE getPointerToFunction(unsigned cluster_id, uint64_t phys_addr, std::function<llvm::Function*(llvm::Module*)> generator, bool dump_enabled) {
        auto it = this->func_map.find(phys_addr);
        if (it == this->func_map.end()) {
            return (FTYPE)MCJIT_helper::getPointerToFunction_(cluster_id, phys_addr, generator, dump_enabled);
        } else {
            return (FTYPE)it->second.f_ptr;
        }
    }
    /**
     * remove all compiled functions for a given cluster
     *
     * @param cluster_id
     */
    void flush_entries(unsigned cluster_id){
        for(std::pair<long unsigned int,MCJIT_block>& e: func_map){
            if ( llvm::ExecutionEngine** p1 = boost::get<llvm::ExecutionEngine*>(&(e.second.mod_eng)))
                delete(*p1);
            else if ( llvm::Module** p2 = boost::get<llvm::Module*>(&(e.second.mod_eng)))
                delete(*p2);
        }
        func_map.clear();
    }
    /**
     * remove a specific function (at a physical address) for a given cluster
     *
     * @param cluster_id    id of the cluster the code is JIT'ed for
     * @param phys_addr     physical address of the JIT#ed code
     */
    void remove_entry(unsigned cluster_id, uint64_t phys_addr){
        auto it =func_map.find(phys_addr);
        if(it!=func_map.end()){
            MCJIT_block& e = it->second;
            if ( llvm::ExecutionEngine** p1 = boost::get<llvm::ExecutionEngine*>( &e.mod_eng ) )
                delete(*p1);
            else if ( llvm::Module** p2 = boost::get<llvm::Module*>( &e.mod_eng ) )
                delete(*p2);
            func_map.erase(it);
        }
    }
    /**
     * get the number of JIT'ed code blocks for a given cluster
     * @param cluster_id
     * @return  the number ot entries in the lookup table
     */
    size_t size(unsigned cluster_id){
        return func_map.size();
    }
    /**
     * get an IR builder
     *
     * @return  the IR builder reference
     */
    llvm::IRBuilder<>& builder() { return *bldr.get();}
protected:
    uint64_t getPointerToFunction_(unsigned cluster_id, uint64_t phys_addr, std::function<llvm::Function*(llvm::Module*)> generator, bool dumpEnabled);
    llvm::LLVMContext &context;
    Loki::AssocVector<uint64_t, MCJIT_block> func_map;
    std::unique_ptr<llvm::IRBuilder<> > bldr;
};
}
}
#endif /* _MCJITHELPER_H_ */
