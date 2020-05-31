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

#include <iss/iss.h>
#include <iss/llvm/vm_base.h>
#include <iss/vm_plugin.h>
#include <util/logging.h>

namespace iss {
namespace llvm {

using namespace std;
using namespace ::llvm;

cl::opt<uint32_t> LikelyBranchWeight("likely-branch-weight", cl::Hidden, cl::init(64),
                                           cl::desc("Weight of the branch likely to be taken (default = 64)"));
cl::opt<uint32_t> UnlikelyBranchWeight("unlikely-branch-weight", cl::Hidden, cl::init(4),
                                             cl::desc("Weight of the branch unlikely to be taken (default = 4)"));
#define INT_TYPE(L) Type::getIntNTy(mod->getContext(), L)
#define FLOAT_TYPE Type::getFloatTy(mod->getContext())
#define DOUBLE_TYPE Type::getDoubleTy(mod->getContext())
#define VOID_TYPE Type::getVoidTy(mod->getContext())
#define THIS_PTR_TYPE Type::getIntNPtrTy(mod->getContext(), 8)
#define FDECLL(NAME, RET, ...)                                                                                         \
    Function *NAME##_func = CurrentModule->getFunction(#NAME);                                                         \
    if (!NAME##_func) {                                                                                                \
        std::vector<Type *> NAME##_args{__VA_ARGS__};                                                                  \
        FunctionType *NAME##_type = FunctionType::get(RET, NAME##_args, false);                                        \
        NAME##_func = Function::Create(NAME##_type, GlobalValue::ExternalLinkage, #NAME, CurrentModule);               \
        NAME##_func->setCallingConv(CallingConv::C);                                                                   \
    }

#define FDECL(NAME, RET, ...)                                                                                          \
    std::vector<Type *> NAME##_args{__VA_ARGS__};                                                                      \
    FunctionType *NAME##_type = FunctionType::get(RET, NAME##_args, false);                                      \
    mod->getOrInsertFunction(#NAME, NAME##_type);

using namespace llvm;

void add_functions_2_module(Module *mod) {
    //    FDECL(get_reg, INT_TYPE(64), THIS_PTR_TYPE, INT_TYPE(16));
    //    FDECL(set_reg, VOID_TYPE, THIS_PTR_TYPE, INT_TYPE(16), INT_TYPE(64));
    //    FDECL(get_flag, INT_TYPE(1), THIS_PTR_TYPE, INT_TYPE(16));
    //    FDECL(set_flag, VOID_TYPE, THIS_PTR_TYPE, INT_TYPE(16), INT_TYPE(1));
    //    FDECL(update_flags, VOID_TYPE, THIS_PTR_TYPE, INT_TYPE(16), INT_TYPE(64), INT_TYPE(64));
    FDECL(fetch, INT_TYPE(8), THIS_PTR_TYPE, INT_TYPE(32), INT_TYPE(32), INT_TYPE(64), INT_TYPE(32),
          INT_TYPE(8)->getPointerTo());
    FDECL(fetch_dbg, INT_TYPE(8), THIS_PTR_TYPE, INT_TYPE(32), INT_TYPE(32), INT_TYPE(64), INT_TYPE(32),
          INT_TYPE(8)->getPointerTo());
    FDECL(read_mem, INT_TYPE(8), THIS_PTR_TYPE, INT_TYPE(32), INT_TYPE(32), INT_TYPE(64), INT_TYPE(32),
          INT_TYPE(8)->getPointerTo());
    FDECL(write_mem, INT_TYPE(8), THIS_PTR_TYPE, INT_TYPE(32), INT_TYPE(32), INT_TYPE(64), INT_TYPE(32),
          INT_TYPE(8)->getPointerTo());
    FDECL(read_mem_dbg, INT_TYPE(8), THIS_PTR_TYPE, INT_TYPE(32), INT_TYPE(32), INT_TYPE(64), INT_TYPE(32),
          INT_TYPE(8)->getPointerTo());
    FDECL(write_mem_dbg, INT_TYPE(8), THIS_PTR_TYPE, INT_TYPE(32), INT_TYPE(32), INT_TYPE(64), INT_TYPE(32),
          INT_TYPE(8)->getPointerTo());
    FDECL(enter_trap, INT_TYPE(64), THIS_PTR_TYPE, INT_TYPE(64), INT_TYPE(64));
    FDECL(leave_trap, INT_TYPE(64), THIS_PTR_TYPE, INT_TYPE(64));
    FDECL(wait, VOID_TYPE, THIS_PTR_TYPE, INT_TYPE(64));
    FDECL(print_string, VOID_TYPE, THIS_PTR_TYPE, INT_TYPE(8)->getPointerTo());
    FDECL(print_disass, VOID_TYPE, THIS_PTR_TYPE, INT_TYPE(64), INT_TYPE(8)->getPointerTo());
    FDECL(pre_instr_sync, VOID_TYPE, THIS_PTR_TYPE);
    FDECL(notify_phase, VOID_TYPE, THIS_PTR_TYPE, INT_TYPE(32));
    FDECL(call_plugin, VOID_TYPE, THIS_PTR_TYPE, INT_TYPE(64));
}
}
}
