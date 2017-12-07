////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2017, MINRES Technologies GmbH
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Contributors:
//       eyck@minres.com - initial API and implementation
////////////////////////////////////////////////////////////////////////////////

#include <iss/iss.h>
#include <iss/vm_base.h>
#include <util/logging.h>

using namespace std;

namespace iss {
namespace vm {

llvm::cl::opt<uint32_t> LikelyBranchWeight("likely-branch-weight", llvm::cl::Hidden, llvm::cl::init(64),
                                           llvm::cl::desc("Weight of the branch likely to be taken (default = 64)"));
llvm::cl::opt<uint32_t> UnlikelyBranchWeight("unlikely-branch-weight", llvm::cl::Hidden, llvm::cl::init(4),
                                             llvm::cl::desc("Weight of the branch unlikely to be taken (default = 4)"));

#define INT_TYPE(L) IntegerType::get(mod->getContext(), L)
#define VOID_TYPE Type::getVoidTy(mod->getContext())
#define THIS_PTR_TYPE INT_TYPE(8)->getPointerTo()
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
    FunctionType *NAME##_type = llvm::FunctionType::get(RET, NAME##_args, false);                                      \
    mod->getOrInsertFunction(#NAME, NAME##_type);

using namespace llvm;

void add_functions_2_module(Module *mod) {
    // Type* voidType = Type::getVoidTy(CurrentModule->getContext());
    FDECL(get_reg, INT_TYPE(64), THIS_PTR_TYPE, INT_TYPE(16));
    FDECL(set_reg, VOID_TYPE, THIS_PTR_TYPE, INT_TYPE(16), INT_TYPE(64));
    FDECL(get_flag, INT_TYPE(1), THIS_PTR_TYPE, INT_TYPE(16));
    FDECL(set_flag, VOID_TYPE, THIS_PTR_TYPE, INT_TYPE(16), INT_TYPE(1));
    FDECL(update_flags, VOID_TYPE, THIS_PTR_TYPE, INT_TYPE(16), INT_TYPE(64), INT_TYPE(64));
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
    FDECL(post_instr_sync, VOID_TYPE, THIS_PTR_TYPE);
}

}
}

#include <iss/arch_if.h>
#include <iss/iss.h>
#include <util/logging.h>

using this_t = uint8_t *;
// Use default logger
// static el::Logger* get_logger(){
//    static el::Logger* logger = el::Loggers::getLogger(disass, true);
//    return logger;
//}

extern "C" {
uint64_t get_reg(this_t iface, int16_t idx) {
    std::vector<uint8_t> data(8, 0);
    ((iss::arch_if *)iface)->get_reg(idx, data);
#ifdef EXEC_LOGGING
    LOG(TRACE) << "EXEC: read reg " << idx << " of core " << iface << " getting value 0x" << hex << res << dec;
#endif
    return *(reinterpret_cast<uint64_t *>(&data[0]));
}

void set_reg(this_t iface, int16_t idx, uint64_t value) {
#ifdef EXEC_LOGGING
    LOG(TRACE) << "EXEC: write reg " << idx << " of core " << iface << " with value 0x" << hex << value << dec;
#endif
    std::vector<uint8_t> data(8, 0);
    *(reinterpret_cast<uint64_t *>(&data[0])) = value;
    ((iss::arch_if *)iface)->set_reg(idx, data);
}

bool get_flag(this_t iface, int16_t flag) {
    bool res = ((iss::arch_if *)iface)->get_flag(flag);
#ifdef EXEC_LOGGING
    LOG(TRACE) << "EXEC: read flag " << flag << " of core " << iface << " getting value 0x" << hex << res << dec;
#endif
    return res;
}

void set_flag(this_t iface, int16_t flag, bool value) {
#ifdef EXEC_LOGGING
    LOG(TRACE) << "EXEC: write flag " << flag << " of core " << iface << " with value 0x" << value;
#endif
    ((iss::arch_if *)iface)->set_flag(flag, value);
}

void update_flags(this_t iface, int16_t op, uint64_t opr1, uint64_t opr2) {
    ((iss::arch_if *)iface)->update_flags((iss::arch_if::operations)op, opr1, opr2);
}

uint8_t fetch(this_t iface, uint32_t addr_type, uint32_t space, uint64_t addr, uint32_t length, uint8_t *data) {
    return ((iss::arch_if *)iface)->read(iss::addr_t{iss::FETCH | addr_type, space, addr}, length, data);
}

uint8_t fetch_dbg(this_t iface, uint32_t addr_type, uint32_t space, uint64_t addr, uint32_t length, uint8_t *data) {
    return ((iss::arch_if *)iface)->read(iss::addr_t{iss::DEBUG_FETCH | addr_type, space, addr}, length, data);
}
uint8_t read_mem(this_t iface, uint32_t addr_type, uint32_t space, uint64_t addr, uint32_t length, uint8_t *data) {
    return ((iss::arch_if *)iface)->read(iss::addr_t{iss::READ | addr_type, space, addr}, length, data);
}

uint8_t write_mem(this_t iface, uint32_t addr_type, uint32_t space, uint64_t addr, uint32_t length, uint8_t *data) {
#ifdef EXEC_LOGGING
    LOG(TRACE) << "EXEC: write mem " << (unsigned)type << " of core " << iface << " at addr 0x" << hex << addr
               << " with value 0x" << data << dec << " of len " << length;
#endif
    return ((iss::arch_if *)iface)->write(iss::addr_t{iss::WRITE | addr_type, space, addr}, length, data);
}

uint8_t read_mem_dbg(this_t iface, uint32_t addr_type, uint32_t space, uint64_t addr, uint32_t length, uint8_t *data) {
    return ((iss::arch_if *)iface)->read(iss::addr_t{iss::DEBUG_READ | addr_type, space, addr}, length, data);
}

uint8_t write_mem_dbg(this_t iface, uint32_t addr_type, uint32_t space, uint64_t addr, uint32_t length, uint8_t *data) {
    return ((iss::arch_if *)iface)->write(iss::addr_t{iss::DEBUG_WRITE | addr_type, space, addr}, length, data);
}

uint64_t enter_trap(this_t iface, uint64_t flags, uint64_t addr) {
    return ((iss::arch_if *)iface)->enter_trap(flags, addr);
}

uint64_t leave_trap(this_t iface, uint64_t flags) { return ((iss::arch_if *)iface)->leave_trap(flags); }

void wait(this_t iface, uint64_t flags) { ((iss::arch_if *)iface)->wait_until(flags); }

void print_string(this_t iface, char *str) { LOG(DEBUG) << "[EXEC] " << str; }

void print_disass(this_t iface, uint64_t pc, char *str) {
    ((iss::arch_if *)iface)->disass_output(pc, str);
}

void pre_instr_sync(this_t iface) {
    iss::vm_if *vm = reinterpret_cast<iss::vm_if *>(iface);
    vm->pre_instr_sync();
}

void post_instr_sync(this_t iface) { ((iss::vm_if *)iface)->post_instr_sync(); }
}

