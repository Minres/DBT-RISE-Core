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

using namespace std;
using namespace llvm;
namespace iss {

namespace llvm {

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


#include <iss/arch_if.h>
#include <iss/iss.h>
#include <util/logging.h>

using namespace iss;
//using this_t = uint8_t *;
using arch_if_ptr_t = arch_if *;
using vm_if_ptr_t = vm_if *;
using vm_plugin_ptr_t = vm_plugin *;

extern "C" {

uint8_t fetch(arch_if_ptr_t iface, uint32_t addr_type, uint32_t space, uint64_t addr, uint32_t length, uint8_t *data) {
    return iface->read((address_type)addr_type, access_type::FETCH, (uint16_t)space, addr, length, data);
}

uint8_t fetch_dbg(arch_if_ptr_t iface, uint32_t addr_type, uint32_t space, uint64_t addr, uint32_t length, uint8_t *data) {
    return iface->read((address_type)addr_type, access_type::DEBUG_FETCH, (uint16_t)space, addr, length, data);
}

uint8_t read_mem(arch_if_ptr_t iface, uint32_t addr_type, uint32_t space, uint64_t addr, uint32_t length, uint8_t *data) {
    return iface->read((address_type)addr_type, access_type::READ, (uint16_t)space, addr, length, data);
}

uint8_t write_mem(arch_if_ptr_t iface, uint32_t addr_type, uint32_t space, uint64_t addr, uint32_t length, uint8_t *data) {
#ifdef EXEC_LOGGING
    LOG(TRACE) << "EXEC: write mem " << (unsigned)type << " of core " << iface << " at addr 0x" << hex << addr
               << " with value 0x" << data << dec << " of len " << length;
#endif
    return iface->write((address_type)addr_type, access_type::WRITE, (uint16_t)space, addr, length, data);
}

uint8_t read_mem_dbg(arch_if_ptr_t iface, uint32_t addr_type, uint32_t space, uint64_t addr, uint32_t length, uint8_t *data) {
    return iface->read((address_type)addr_type, access_type::DEBUG_READ, (uint16_t)space, addr, length, data);
}

uint8_t write_mem_dbg(arch_if_ptr_t iface, uint32_t addr_type, uint32_t space, uint64_t addr, uint32_t length, uint8_t *data) {
    return iface->write((address_type)addr_type, access_type::DEBUG_WRITE, (uint16_t)space, addr, length, data);
}

uint64_t enter_trap(arch_if_ptr_t iface, uint64_t flags, uint64_t addr) {
    return iface->enter_trap(flags, addr);
}

uint64_t leave_trap(arch_if_ptr_t iface, uint64_t flags) { return iface->leave_trap(flags); }

void wait(arch_if_ptr_t iface, uint64_t flags) { iface->wait_until(flags); }

void print_string(arch_if_ptr_t iface, char *str) { LOG(DEBUG) << "[EXEC] " << str; }

void print_disass(arch_if_ptr_t iface, uint64_t pc, char *str) { iface->disass_output(pc, str); }

void pre_instr_sync(vm_if_ptr_t iface) { iface->pre_instr_sync(); }

void notify_phase(arch_if_ptr_t iface, uint32_t phase) {
    iface->notify_phase((arch_if::exec_phase)phase);
}

void call_plugin(vm_plugin_ptr_t iface, uint64_t instr_info) {
    iface->callback(instr_info_t(instr_info));
}
}
