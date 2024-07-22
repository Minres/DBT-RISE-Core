/*******************************************************************************
 * Copyright (C) 2023 MINRES Technologies GmbH
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
 *       alex@minres.com - initial implementation
 ******************************************************************************/

#ifndef ASMJIT_VM_BASE_H_
#define ASMJIT_VM_BASE_H_

#include <asmjit/x86/x86compiler.h>
#include <asmjit/x86/x86operand.h>
#include <cassert>
#include <cstddef>
#include <cstdlib>
#include <fmt/core.h>
#include <fmt/format.h>
#include <iss/arch/traits.h>
#include <iss/arch_if.h>
#include <iss/debugger/target_adapter_base.h>
#include <iss/debugger_if.h>
#include <iss/vm_if.h>
#include <iss/vm_plugin.h>
#include <iterator>
#include <stdexcept>
#include <type_traits>
#include <util/ities.h>
#include <util/logging.h>

#include "jit_helper.h"
#include <asmjit/asmjit.h>
extern "C" {
#include <iss/vm_jit_funcs.h>
}
#include <array>
#include <chrono>
#include <iostream>
#include <map>
#include <sstream>
#include <stack>
#include <unordered_map>
#include <utility>
#include <variant>
#include <vector>

namespace iss {

namespace asmjit {
using namespace ::asmjit;
struct dGp {
    x86::Gp upper;
    x86::Gp lower;
    dGp(x86::Compiler& cc)
    : upper(cc.newInt64())
    , lower(cc.newInt64()) {}
    dGp() = delete;
};
using x86_reg_t = std::variant<x86::Gp, dGp>;

enum continuation_e { CONT, BRANCH, FLUSH, TRAP };
enum last_branch_e { NO_JUMP = 0, KNOWN_JUMP = 1, UNKNOWN_JUMP = 2 };
enum operation { add, sub, band, bor, bxor, shl, sar, shr };
enum complex_operation { smul, umul, sumul, sdiv, udiv, srem, urem };
enum comparison_operation { land, lor, eq, ne, lt, ltu, gt, gtu, lte, lteu, gte, gteu };
enum unary_operation { lnot, inc, dec, bnot, neg };

template <typename ARCH> class vm_base : public debugger_if, public vm_if {
    struct plugin_entry {
        sync_type sync;
        vm_plugin& plugin;
        void* plugin_ptr; // FIXME: hack
    };

public:
    using reg_e = typename arch::traits<ARCH>::reg_e;
    using sr_flag_e = typename arch::traits<ARCH>::sreg_flag_e;
    using virt_addr_t = typename arch::traits<ARCH>::virt_addr_t;
    using phys_addr_t = typename arch::traits<ARCH>::phys_addr_t;
    using addr_t = typename arch::traits<ARCH>::addr_t;
    using code_word_t = typename arch::traits<ARCH>::code_word_t;
    using mem_type_e = typename arch::traits<ARCH>::mem_type_e;
    using traits = typename arch::traits<ARCH>::traits;

    using dbg_if = iss::debugger_if;
    constexpr static unsigned blk_size = 128; // std::numeric_limits<unsigned>::max();

    arch_if* get_arch() override { return &core; };

    constexpr unsigned int get_reg_width(int idx) const {
        return idx < 0 ? arch::traits<ARCH>::NUM_REGS : arch::traits<ARCH>::reg_bit_widths[(reg_e)idx];
    }

    template <typename T> inline T get_reg(unsigned r) {
        std::vector<uint8_t> res(sizeof(T), 0);
        uint8_t* reg_base = core.get_regs_base_ptr() + arch::traits<ARCH>::reg_byte_offsets[r];
        auto size = arch::traits<ARCH>::reg_bit_widths[r] / 8;
        std::copy(reg_base, reg_base + size, res.data());
        return *reinterpret_cast<T*>(&res[0]);
    }

    using func_ptr = uint64_t (*)(uint8_t*, void*, void*);

    int start(uint64_t icount = std::numeric_limits<uint64_t>::max(), bool dump = false,
              finish_cond_e cond = finish_cond_e::ICOUNT_LIMIT | finish_cond_e::JUMP_TO_SELF) override {
        int error = 0;
        if(this->debugging_enabled())
            sync_exec = PRE_SYNC;
        auto start = std::chrono::high_resolution_clock::now();
        virt_addr_t pc(iss::access_type::DEBUG_FETCH, 0, get_reg<typename arch::traits<ARCH>::addr_t>(arch::traits<ARCH>::PC));
        CPPLOG(INFO) << "Start at 0x" << std::hex << pc.val << std::dec;
        try {
            continuation_e cont = CONT;

            std::function<void(jit_holder&)> generator{[this, &pc, &cont](jit_holder& jh) -> void {
                gen_block_prologue(jh);
                cont = translate(pc, jh);
                gen_block_epilogue(jh);
                // move local disass collection to global collection by appending
                this->global_disass_collection.insert(this->global_disass_collection.end(), jh.disass_collection.begin(),
                                                      jh.disass_collection.end());
                jh.disass_collection.clear();
            }};
            // explicit std::function to allow use as reference in call below
            // std::function<Function*(Module*)> gen_ref(std::ref(generator));
            translation_block *last_tb = nullptr, *cur_tb = nullptr;
            uint32_t last_branch = std::numeric_limits<uint32_t>::max();
            arch_if* const arch_if_ptr = static_cast<arch_if*>(&core);
            vm_if* const vm_if_ptr = static_cast<vm_if*>(this);
            while(!core.should_stop() && core.get_icount() < icount) {
                try {
                    // translate into physical address
                    phys_addr_t pc_p(pc.access, pc.space, pc.val);
                    if(this->core.has_mmu())
                        pc_p = this->core.virt2phys(pc);
                    // check if we have the block already compiled
                    auto it = this->func_map.find(pc_p.val);
                    if(it == this->func_map.end()) { // if not generate and compile it
                        auto res = func_map.insert(
                            std::make_pair(pc_p.val, iss::asmjit::getPointerToFunction(cluster_id, pc_p.val, generator, dump)));
                        it = res.first;
                    }
                    cur_tb = &(it->second);
                    // if we have a previous block link the just compiled one as successor of the last tb
                    if(last_tb && last_branch < 2 && last_tb->cont[last_branch] == nullptr)
                        last_tb->cont[last_branch] = cur_tb;
                    do {
                        // execute the compiled function
                        pc.val = reinterpret_cast<func_ptr>(cur_tb->f_ptr)(regs_base_ptr, arch_if_ptr, vm_if_ptr);
                        if(core.should_stop())
                            break;
                        // update last state
                        last_tb = cur_tb;
                        last_branch = core.get_last_branch();
                        auto cur_icount = core.get_icount();
                        // if the current tb has a successor assign to current tb
                        if(last_branch < 2 && cur_tb->cont[last_branch] != nullptr && cur_icount < icount)
                            cur_tb = cur_tb->cont[last_branch];
                        else // if not we need to compile one
                            cur_tb = nullptr;
                    } while(cur_tb != nullptr);
                    if(cont == FLUSH)
                        func_map.clear();

                } catch(trap_access& ta) {
                    pc.val = core.enter_trap(ta.id, ta.addr, 0);
                }
#ifndef NDEBUG
                CPPLOG(TRACE) << "continuing  @0x" << std::hex << pc << std::dec;
#endif
                // break; //for testing, only exec first block and end sim
            }
        } catch(simulation_stopped& e) {
            CPPLOG(INFO) << "ISS execution stopped with status 0x" << std::hex << e.state << std::dec;
            if(e.state != 1)
                error = e.state;
        } catch(decoding_error& e) {
            CPPLOG(ERR) << "ISS execution aborted at address 0x" << std::hex << e.addr << std::dec;
            error = -1;
        }
        auto end = std::chrono::high_resolution_clock::now(); // end measurement
        // here
        auto elapsed = end - start;
        auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
        CPPLOG(INFO) << "Executed " << core.get_icount() << " instructions in " << func_map.size() << " code blocks during " << millis
                     << "ms resulting in " << (core.get_icount() * 0.001 / millis) << "MIPS";
        return error;
    }

    void reset() override { core.reset(); }

    void reset(uint64_t address) override { core.reset(address); }

    void pre_instr_sync() override {
        uint64_t pc = get_reg<typename arch::traits<ARCH>::addr_t>(arch::traits<ARCH>::PC);
        tgt_adapter->check_continue(pc);
    }

protected:
    continuation_e translate(virt_addr_t& pc, jit_holder& jh) {
        unsigned cur_blk = 0;
        virt_addr_t cur_pc = pc;
        phys_addr_t phys_pc(pc.access, pc.space, pc.val);
        if(this->core.has_mmu())
            phys_pc = this->core.virt2phys(pc);
        std::pair<virt_addr_t, phys_addr_t> cur_pc_mark(pc, phys_pc);
        unsigned int num_inst = 0;
        continuation_e cont = CONT;
        try {
            while(cont == CONT && cur_blk < blk_size) {
                cont = gen_single_inst_behavior(cur_pc, num_inst, jh);
                cur_blk++;
            }
            assert(cur_pc_mark.first.val <= cur_pc.val);
            return cont;
        } catch(trap_access& ta) {
            if(cur_pc_mark.first.val <= cur_pc.val) {
                return cont;
            } else
                throw ta;
        }
        return cont;
    }
    virtual continuation_e gen_single_inst_behavior(virt_addr_t&, unsigned int&, jit_holder&) = 0;
    virtual void gen_block_prologue(jit_holder&) = 0;
    virtual void gen_block_epilogue(jit_holder&) = 0;

    explicit vm_base(ARCH& core, unsigned core_id = 0, unsigned cluster_id = 0)
    : core(core)
    , core_id(core_id)
    , cluster_id(cluster_id)
    , regs_base_ptr(core.get_regs_base_ptr())
    , sync_exec(NO_SYNC)
    , tgt_adapter(nullptr) {
        sync_exec = static_cast<sync_type>(sync_exec | core.needed_sync());
    }

    ~vm_base() override {
        delete tgt_adapter;
        for(auto& each : global_disass_collection) {
            free(each);
        }
        global_disass_collection.clear();
    }

    void register_plugin(vm_plugin& plugin) override {
        if(plugin.registration("1.0", *this)) {
            plugins.push_back(plugin_entry{plugin.get_sync(), plugin, &plugin});
        }
    }
    void gen_sync(jit_holder& jh, sync_type s, unsigned inst_id) {
        if(plugins.size() /*or debugger*/)
            write_back(jh);
        for(plugin_entry e : plugins) {
            if(e.sync & s) {
                iss::instr_info_t iinfo{cluster_id, core_id, inst_id, s};
                InvokeNode* call_plugin_node;
                jh.cc.comment("//Plugin call:");
                jh.cc.invoke(&call_plugin_node, &call_plugin, FuncSignature::build<void, void*, uint64_t>());
                call_plugin_node->setArg(0, e.plugin_ptr);
                call_plugin_node->setArg(1, iinfo.backing.val);
            }
        }
        // TODO: handle Debugger
    }

    inline void gen_wait(jit_holder& jh, unsigned type) {
        InvokeNode* call_wait;
        jh.cc.comment("//gen_wait");
        jh.cc.invoke(&call_wait, &wait, FuncSignature::build<void, void*, uint64_t>());
        call_wait->setArg(0, this->get_arch());
        call_wait->setArg(1, type);
    }
    inline void gen_leave(jit_holder& jh, unsigned lvl) {
        InvokeNode* call_leave;
        jh.cc.comment("//gen_leave");
        jh.cc.invoke(&call_leave, &leave_trap, FuncSignature::build<void, void*, uint64_t>());
        call_leave->setArg(0, this->get_arch());
        call_leave->setArg(1, lvl);
        mov(jh.cc, get_ptr_for(jh, traits::LAST_BRANCH), static_cast<int>(UNKNOWN_JUMP));
        jh.next_pc = load_reg_from_mem_Gp(jh, traits::NEXT_PC);
    }
    template <typename T, typename = std::enable_if_t<std::is_integral_v<T>>> inline void gen_set_tval(jit_holder& jh, T new_tval) {
        mov(jh.cc, jh.tval, new_tval);
    }
    inline void gen_set_tval(jit_holder& jh, x86_reg_t _new_tval) {
        if(std::holds_alternative<x86::Gp>(_new_tval)) {
            x86::Gp new_tval = std::get<x86::Gp>(_new_tval);
            if(new_tval.size() < 8)
                new_tval = gen_ext_Gp(jh.cc, new_tval, 64, false);
            mov(jh.cc, jh.tval, new_tval);
        } else {
            throw std::runtime_error("Variant not supported in gen_set_tval");
        }
    }

    ARCH& core;
    unsigned core_id = 0;
    unsigned cluster_id = 0;
    uint8_t* regs_base_ptr;
    sync_type sync_exec;
    std::unordered_map<uint64_t, translation_block> func_map;
    iss::debugger::target_adapter_base* tgt_adapter;
    std::vector<plugin_entry> plugins;
    std::vector<char*> global_disass_collection;

    // Asmjit generator functions for commonly used expressions
    void write_back(jit_holder& jh) {
        write_reg_to_mem(jh, jh.pc, traits::PC);
        write_reg_to_mem(jh, jh.next_pc, traits::NEXT_PC);
    }
    inline void mov(x86::Compiler& cc, x86_reg_t _dest, x86_reg_t _source) {
        if(std::holds_alternative<x86::Gp>(_dest) && std::holds_alternative<x86::Gp>(_source)) {
            x86::Gp dest = std::get<x86::Gp>(_dest);
            x86::Gp source = std::get<x86::Gp>(_source);
            assert(dest.size() == source.size());
            cc.mov(dest, source);
        } else {
            throw std::runtime_error("Variant not implemented in mov");
        }
    }
    template <typename T, typename = std::enable_if_t<std::is_integral<T>::value>>
    inline void mov(x86::Compiler& cc, x86_reg_t dest, T source) {
        if(std::holds_alternative<x86::Gp>(dest)) {
            cc.mov(std::get<x86::Gp>(dest), source);
        } else {
            throw std::runtime_error("Variant not implemented in mov (Integral)");
        }
    }
    inline void mov(x86::Compiler& cc, x86_reg_t dest, x86::Mem source) {
        if(std::holds_alternative<x86::Gp>(dest)) {
            cc.mov(std::get<x86::Gp>(dest), source);
        } else {
            throw std::runtime_error("Variant not implemented in mov (Mem)");
        }
    }
    inline void mov(x86::Compiler& cc, x86::Mem dest, x86_reg_t source) {
        if(std::holds_alternative<x86::Gp>(source)) {
            cc.mov(dest, std::get<x86::Gp>(source));
        } else {
            throw std::runtime_error("Variant not implemented in mov (Mem as dest)");
        }
    }
    template <typename T, typename = std::enable_if_t<std::is_integral<T>::value>>
    inline void mov(x86::Compiler& cc, x86::Mem dest, T source) {
        // immediates larger than 32 bits need to be moved into a register first
        if(sizeof(T) <= 4) {
            cc.mov(dest, source);
        } else {
            x86::Gp imm_reg = get_reg_Gp(cc, sizeof(T) * 8);
            cc.mov(imm_reg, source);
            mov(cc, dest, imm_reg);
        }
    }
    inline void mov(x86::Compiler& cc, x86::Mem dest, x86::Gp source) { cc.mov(dest, source); }

    inline void cmp(x86::Compiler& cc, x86_reg_t a, x86_reg_t b) {
        if(std::holds_alternative<x86::Gp>(a) && std::holds_alternative<x86::Gp>(b)) {
            cc.cmp(std::get<x86::Gp>(a), std::get<x86::Gp>(b));
        } else {
            throw std::runtime_error("Variant not implemented in cmp");
        }
    }
    template <typename T, typename = std::enable_if_t<std::is_integral<T>::value>> inline void cmp(x86::Compiler& cc, x86_reg_t _a, T b) {
        if(std::holds_alternative<x86::Gp>(_a)) {
            x86::Gp a = std::get<x86::Gp>(_a);
            // cmp only allows 32-bit immediates
            if(sizeof(b) <= 4)
                cc.cmp(a, b);
            else if(sizeof(b) <= 8) {
                x86::Gp b_reg = get_reg_Gp(cc, sizeof(b) * 8);
                cc.mov(b_reg, b);
                cc.cmp(a, b_reg);
            } else {
                throw std::runtime_error("Size not allowed in cmp (Integral)");
            }

        } else {
            throw std::runtime_error("Variant not implemented in cmp (Integral)");
        }
    }
    inline void cmp(x86::Compiler& cc, x86_reg_t a, x86::Mem b) {
        if(std::holds_alternative<x86::Gp>(a)) {
            cc.cmp(std::get<x86::Gp>(a), b);
        } else {
            throw std::runtime_error("Variant not implemented in cmp (Mem)");
        }
    }
    inline void cmp(x86::Compiler& cc, x86::Mem a, x86_reg_t b) {
        if(std::holds_alternative<x86::Gp>(b)) {
            cc.cmp(a, std::get<x86::Gp>(b));
        } else {
            throw std::runtime_error("Variant not implemented in cmp (Mem as dest)");
        }
    }
    template <typename T, typename = std::enable_if_t<std::is_integral_v<T> || std::is_same_v<T, x86::Mem> || std::is_same_v<T, x86::Gp>>>
    inline void cmp(x86::Compiler& cc, x86::Mem a, T b) {
        cc.cmp(a, b);
    }

    x86::Mem get_ptr_for(jit_holder& jh, unsigned idx) {
        switch(traits::reg_bit_widths[idx]) {
        case 8:
            return x86::ptr_8(jh.regs_base_ptr, traits::reg_byte_offsets[idx]);
        case 16:
            return x86::ptr_16(jh.regs_base_ptr, traits::reg_byte_offsets[idx]);
        case 32:
            return x86::ptr_32(jh.regs_base_ptr, traits::reg_byte_offsets[idx]);
        case 64:
            return x86::ptr_64(jh.regs_base_ptr, traits::reg_byte_offsets[idx]);
        default:
            throw std::runtime_error("Invalid reg size in get_ptr_for");
        }
    }
    x86_reg_t get_reg(x86::Compiler& cc, unsigned size, bool is_signed = true) {
        if(is_signed)
            switch(size) {
            case 8:
                return cc.newInt8();
            case 16:
                return cc.newInt16();
            case 32:
                return cc.newInt32();
            case 64:
                return cc.newInt64();
            case 128:
                return dGp(cc);
            default:
                throw std::runtime_error("Invalid reg size in get_reg");
            }
        else
            switch(size) {
            case 8:
                return cc.newUInt8();
            case 16:
                return cc.newUInt16();
            case 32:
                return cc.newUInt32();
            case 64:
                return cc.newUInt64();
            case 128:
                return dGp(cc);
            default:
                throw std::runtime_error("Invalid reg size in get_reg");
            }
    }
    inline x86::Gp get_reg_Gp(x86::Compiler& cc, unsigned size, bool is_signed = true) {
        assert(size <= 64);
        return std::get<x86::Gp>(get_reg(cc, size, is_signed));
    }
    inline dGp get_reg_dGp(x86::Compiler& cc, unsigned size, bool is_signed = true) {
        assert(size == 128);
        return std::get<dGp>(get_reg(cc, size, is_signed));
    }

    inline x86_reg_t get_reg_for(x86::Compiler& cc, unsigned idx) { return get_reg(cc, traits::reg_bit_widths[idx]); }
    inline x86::Gp get_reg_for_Gp(x86::Compiler& cc, unsigned idx) { return get_reg_Gp(cc, traits::reg_bit_widths[idx]); }
    inline dGp get_reg_for_dGp(x86::Compiler& cc, unsigned idx) { return get_reg_dGp(cc, traits::reg_bit_widths[idx]); }

    inline x86_reg_t load_reg_from_mem(jit_holder& jh, unsigned idx) {
        auto ptr = get_ptr_for(jh, idx);
        auto reg = get_reg_for(jh.cc, idx);
        mov(jh.cc, reg, ptr);
        return reg;
    }
    inline x86::Gp load_reg_from_mem_Gp(jit_holder& jh, unsigned idx) {
        x86_reg_t return_reg = load_reg_from_mem(jh, idx);
        if(std::holds_alternative<x86::Gp>(return_reg)) {
            return std::get<x86::Gp>(return_reg);
        } else {
            throw std::runtime_error("Invalid variation returned in load_reg_from_mem_Gp");
        }
    }
    inline void write_reg_to_mem(jit_holder& jh, x86_reg_t reg, unsigned idx) {
        // TODO sanitize this, needs checking which variant is in reg
        auto ptr = get_ptr_for(jh, idx);
        mov(jh.cc, ptr, reg);
    }
    template <typename T, typename = std::enable_if_t<std::is_integral<T>::value>>
    inline x86_reg_t gen_ext(x86::Compiler& cc, T val, unsigned target_size, bool is_signed) {
        auto val_reg = get_reg(cc, sizeof(val) * 8, is_signed);
        mov(cc, val_reg, val);
        return gen_ext(cc, val_reg, target_size, is_signed);
    }
    inline x86_reg_t gen_ext(x86::Compiler& cc, x86_reg_t _val, unsigned target_size, bool is_signed) {
        // In case of x86::Gp
        if(std::holds_alternative<x86::Gp>(_val)) {
            auto val = std::get<x86::Gp>(_val);
            if(target_size <= 64) {
                if(val.size() * 8 == target_size) // identity cast
                    return val;
                else if(val.size() * 8 > target_size) // truncation
                    switch(target_size) {
                    case 8:
                        return val.r8();
                    case 16:
                        return val.r16();
                    case 32:
                        return val.r32();
                    default:
                        throw std::runtime_error(fmt::format("Invalid truncation size ({}) in gen_ext", target_size));
                    }
                x86::Gp ret_val = get_reg_Gp(cc, target_size);
                if(is_signed) {
                    if(val.size() == 4 && target_size == 64)
                        cc.movsxd(ret_val, val);
                    else
                        cc.movsx(ret_val, val);
                } else if(val.size() == 1 || val.size() == 2)
                    // movzx can extend 8 and 16 bit values
                    cc.movzx(ret_val, val);
                else {
                    assert(val.size() == 4);
                    // from: http://x86asm.net/articles/x86-64-tour-of-intel-manuals/
                    // Perhaps the most surprising fact is that an instruction such as MOV EAX, EBX automatically zeroes
                    // upper 32 bits of RAX register
                    ret_val = get_reg_Gp(cc, val.size() * 8);
                    cc.mov(ret_val, val);
                    switch(target_size) {
                    case 32:
                        return ret_val.r32();
                    case 64:
                        return ret_val.r64();
                    default:
                        throw std::runtime_error("Invalid size in gen_ext");
                    }
                }
                return ret_val;
            } else if(target_size == 128) {
                val = gen_ext_Gp(cc, val, 64, is_signed);
                if(is_signed) {
                    dGp ret_val = dGp(cc);
                    x86::Gp sign_reg = get_reg_Gp(cc, 64);
                    cc.mov(sign_reg, val);
                    cc.sar(sign_reg, 63);
                    cc.mov(ret_val.lower, val);
                    cc.mov(ret_val.upper, sign_reg);
                    return ret_val;
                } else {
                    dGp ret_val = dGp(cc);
                    cc.mov(ret_val.lower, val);
                    cc.mov(ret_val.upper, 0);
                    return ret_val;
                }
            } else {
                throw std::runtime_error(fmt::format("target size {} not supported in gen_ext", target_size));
            }
        }
        // In case of dGp
        else if(std::holds_alternative<dGp>(_val)) {
            auto val = std::get<dGp>(_val);
            if(target_size < 128) {
                // truncation
                if(target_size < 64) {
                    return gen_ext_Gp(cc, val.lower, target_size, is_signed);
                } else if(target_size == 64) {
                    return val.lower;
                } else {
                    val.upper = gen_ext_Gp(cc, val.upper, target_size - 64, is_signed);
                    return val;
                }
            }
            throw std::runtime_error(fmt::format("Does not support gen_ext from dGp to {}", target_size));
            return _val;
        }
        // Should not end here
        else {
            throw std::runtime_error("Invalid variant encountered in gen_ext");
            return _val;
        }
    }
    inline x86::Gp gen_ext_Gp(x86::Compiler& cc, x86_reg_t _val, unsigned size, bool is_signed) {
        assert(size <= 64);
        return std::get<x86::Gp>(gen_ext(cc, _val, size, is_signed));
    }
    inline dGp gen_ext_dGp(x86::Compiler& cc, x86_reg_t _val, unsigned size, bool is_signed) {
        assert(size == 128);
        return std::get<dGp>(gen_ext(cc, _val, size, is_signed));
    }

    inline x86_reg_t gen_read_mem(jit_holder& jh, mem_type_e type, x86_reg_t _addr, uint32_t length) {
        if(std::holds_alternative<x86::Gp>(_addr)) {
            auto addr = std::get<x86::Gp>(_addr);
            x86::Compiler& cc = jh.cc;
            auto ret_reg = cc.newInt32();

            auto mem_type_reg = cc.newInt32();
            cc.mov(mem_type_reg, type);

            auto space_reg = cc.newInt32();
            cc.mov(space_reg, static_cast<uint16_t>(iss::address_type::VIRTUAL));

            auto val_ptr = cc.newUIntPtr();
            cc.mov(val_ptr, read_mem_buf);
            x86::Mem read_res;
            InvokeNode* invokeNode;
            x86::Gp val_reg = get_reg_Gp(cc, length * 8, true);

            switch(length) {
            case 1: {
                cc.invoke(&invokeNode, &read_mem1, FuncSignature::build<uint32_t, uint64_t, uint32_t, uint32_t, uint64_t, uintptr_t>());
                read_res = x86::ptr_8(val_ptr);
                break;
            }
            case 2: {
                cc.invoke(&invokeNode, &read_mem2, FuncSignature::build<uint32_t, uint64_t, uint32_t, uint32_t, uint64_t, uintptr_t>());
                read_res = x86::ptr_16(val_ptr);
                break;
            }
            case 4: {
                cc.invoke(&invokeNode, &read_mem4, FuncSignature::build<uint32_t, uint64_t, uint32_t, uint32_t, uint64_t, uintptr_t>());
                read_res = x86::ptr_32(val_ptr);
                break;
            }
            case 8: {
                cc.invoke(&invokeNode, &read_mem8, FuncSignature::build<uint32_t, uint64_t, uint32_t, uint32_t, uint64_t, uintptr_t>());
                read_res = x86::ptr_64(val_ptr);
                break;
            }
            default:
                throw std::runtime_error(fmt::format("Invalid length ({}) in gen_read_mem", length));
            }

            invokeNode->setRet(0, ret_reg);
            invokeNode->setArg(0, jh.arch_if_ptr);
            invokeNode->setArg(1, space_reg);
            invokeNode->setArg(2, mem_type_reg);
            invokeNode->setArg(3, addr);
            invokeNode->setArg(4, val_ptr);
            cc.cmp(ret_reg, 0);
            cc.jne(jh.trap_entry);

            cc.mov(val_reg, read_res);
            return val_reg;
        }
        // In case of dGp
        else if(std::holds_alternative<dGp>(_addr)) {
            throw std::runtime_error("Variant not supported in gen_read_mem");
            return _addr;
        }
        // Should not end here
        else {
            throw std::runtime_error("Invalid variant in gen_read_mem");
            return _addr;
        }
    }
    inline x86_reg_t gen_read_mem(jit_holder& jh, mem_type_e type, uint64_t addr, uint32_t length) {
        auto addr_reg = get_reg(jh.cc, length * 8, true);
        mov(jh.cc, addr_reg, addr);
        return gen_read_mem(jh, type, addr_reg, length);
    }

    inline void gen_write_mem(jit_holder& jh, mem_type_e type, x86_reg_t _addr, x86_reg_t _val, uint32_t length) {
        if(std::holds_alternative<x86::Gp>(_addr) && std::holds_alternative<x86::Gp>(_val)) {
            auto addr = std::get<x86::Gp>(_addr);
            auto val = std::get<x86::Gp>(_val);
            x86::Compiler& cc = jh.cc;
            assert(val.size() == length);
            auto mem_type_reg = cc.newInt32();
            jh.cc.mov(mem_type_reg, type);
            auto space_reg = cc.newInt32();
            jh.cc.mov(space_reg, static_cast<uint16_t>(iss::address_type::VIRTUAL));
            auto ret_reg = cc.newInt32();
            InvokeNode* invokeNode;
            switch(length) {
            case 1:
                cc.invoke(&invokeNode, &write_mem1, FuncSignature::build<uint32_t, uint64_t, uint32_t, uint32_t, uint64_t, uint8_t>());
                break;
            case 2:
                cc.invoke(&invokeNode, &write_mem2, FuncSignature::build<uint32_t, uint64_t, uint32_t, uint32_t, uint64_t, uint16_t>());
                break;
            case 4:
                cc.invoke(&invokeNode, &write_mem4, FuncSignature::build<uint32_t, uint64_t, uint32_t, uint32_t, uint64_t, uint32_t>());
                break;
            case 8:
                cc.invoke(&invokeNode, &write_mem8, FuncSignature::build<uint32_t, uint64_t, uint32_t, uint32_t, uint64_t, uint64_t>());
                break;
            default:
                throw std::runtime_error("Invalid register size in gen_write_mem");
            }
            invokeNode->setRet(0, ret_reg);
            invokeNode->setArg(0, jh.arch_if_ptr);
            invokeNode->setArg(1, space_reg);
            invokeNode->setArg(2, mem_type_reg);
            invokeNode->setArg(3, addr);
            invokeNode->setArg(4, val);

            cc.cmp(ret_reg, 0);
            cc.jne(jh.trap_entry);
        } else {
            throw std::runtime_error("Invalid variant combination in gen_write_mem");
        }
    }
    inline void gen_write_mem(jit_holder& jh, mem_type_e type, uint64_t addr, x86_reg_t val, uint32_t length) {
        auto addr_reg = jh.cc.newUInt64();
        jh.cc.mov(addr_reg, addr);
        gen_write_mem(jh, type, addr_reg, val, length);
    }
    inline void gen_write_mem(jit_holder& jh, mem_type_e type, uint64_t addr, int64_t val, uint32_t length) {
        auto val_reg = get_reg_Gp(jh.cc, length * 8, true);
        jh.cc.mov(val_reg, val);

        auto addr_reg = jh.cc.newUInt64();
        jh.cc.mov(addr_reg, addr);
        gen_write_mem(jh, type, addr_reg, val_reg, length);
    }
    inline void gen_write_mem(jit_holder& jh, mem_type_e type, x86_reg_t addr, int64_t val, uint32_t length) {
        auto val_reg = get_reg_Gp(jh.cc, length * 8, true);
        jh.cc.mov(val_reg, val);
        gen_write_mem(jh, type, addr, val_reg, length);
    }
    template <typename T, typename = std::enable_if_t<std::is_integral<T>::value>>
    x86_reg_t gen_operation(x86::Compiler& cc, operation op, x86_reg_t _a, T b) {
        if(std::holds_alternative<x86::Gp>(_a)) {
            x86::Gp a = std::get<x86::Gp>(_a);
            return _gen_operation(cc, op, a, b);
        } else if(std::holds_alternative<dGp>(_a)) {
            dGp a = std::get<dGp>(_a);
            dGp ret_val = dGp(cc);
            switch(op) {
            case add:
            case sub:
            case band:
            case bor:
            case bxor: {
                ret_val.upper = _gen_operation(cc, op, a.upper, b >> 64);
                ret_val.lower = _gen_operation(cc, op, a.lower, b & 0xffffffff);
                return ret_val;
            }
            case shl: {
                if(b >= 64) { // shl gets masked to 8 bits, so we need to shift by more than 63 manually
                    assert(b <= 128);
                    cc.mov(ret_val.lower, 0);
                    ret_val.upper = _gen_operation(cc, shl, a.lower, b - 64);
                    return ret_val;
                } else {
                    x86::Gp upper_shifted = _gen_operation(cc, shl, a.upper, b);
                    x86::Gp shifted_out_of_lower = _gen_operation(cc, shr, a.lower, 64 - b);
                    cc.or_(upper_shifted, shifted_out_of_lower);
                    ret_val.upper = upper_shifted;
                    ret_val.lower = _gen_operation(cc, shl, a.lower, b);
                    return ret_val;
                }
            }
            case sar: {
                if(b >= 64) { // sar gets masked to 8 bits, so we need to shift by more than 63 manually
                    assert(b <= 128);
                    cc.mov(ret_val.upper, a.upper);
                    cc.sar(ret_val.upper, 63);
                    ret_val.lower = _gen_operation(cc, sar, a.upper, b - 64);
                    return ret_val;
                } else {
                    x86::Gp lower_shifted = _gen_operation(cc, shr, a.upper, b);
                    x86::Gp shifted_out_of_upper = _gen_operation(cc, shl, a.upper, 64 - b);
                    cc.or_(lower_shifted, shifted_out_of_upper);
                    ret_val.lower = lower_shifted;
                    ret_val.upper = _gen_operation(cc, sar, a.upper, b);
                    return ret_val;
                }
            }
            case shr: {
                if(b >= 64) { // sar gets masked to 8 bits, so we need to shift by more than 63 manually
                    assert(b <= 128);
                    cc.mov(ret_val.upper, 0);
                    ret_val.lower = _gen_operation(cc, shr, a.upper, b - 64);
                    return ret_val;
                } else {
                    x86::Gp lower_shifted = _gen_operation(cc, shr, a.upper, b);
                    x86::Gp shifted_out_of_upper = _gen_operation(cc, shl, a.upper, 64 - b);
                    cc.or_(lower_shifted, shifted_out_of_upper);
                    ret_val.lower = lower_shifted;
                    ret_val.upper = _gen_operation(cc, shr, a.upper, b);
                    return ret_val;
                }
            }
            default:
                throw std::runtime_error(fmt::format("Operation {} not handled in gen_operation for dGp variant", op));
            }
        }

        // Should not end here
        else {
            throw std::runtime_error("Invalid variant in in gen_operation (operation w/ integral)");
            return _a;
        }
    }

    x86_reg_t gen_operation(x86::Compiler& cc, operation op, x86_reg_t _a, x86_reg_t _b) {
        if(std::holds_alternative<x86::Gp>(_a) && std::holds_alternative<x86::Gp>(_b)) {
            x86::Gp a = std::get<x86::Gp>(_a);
            x86::Gp b = std::get<x86::Gp>(_b);
            assert(a.size() == b.size());
            return _gen_operation(cc, op, a, b);
        }
        // Should not end here
        else {
            throw std::runtime_error("Invalid variant combination in in gen_operation (operation)");
            return _a;
        }
    }
    template <typename T, typename = std::enable_if_t<std::is_integral_v<T> || std::is_same_v<T, x86::Gp>>>
    x86::Gp _gen_operation(x86::Compiler& cc, operation op, x86::Gp a, T b) {
        switch(op) {
        case add: {
            cc.add(a, b);
            break;
        }
        case sub: {
            cc.sub(a, b);
            break;
        }
        case band: {
            cc.and_(a, b);
            break;
        }
        case bor: {
            cc.or_(a, b);
            break;
        }
        case bxor: {
            cc.xor_(a, b);
            break;
        }
        case shl: {
            cc.shl(a, b);
            break;
        }
        case sar: {
            cc.sar(a, b);
            break;
        }
        case shr: {
            cc.shr(a, b);
            break;
        }
        default:
            throw std::runtime_error(fmt::format("Current operation {} not supported in gen_operation (operation)", op));
        }
        return a;
    }

    inline void expand_division_operand(x86::Compiler& cc, x86::Gp upper_half, x86::Gp dividend) {
        // This function expands the dividend into the upper half, allowing correct division of negative values
        switch(dividend.size()) {
        case 2:
            cc.cwd(upper_half, dividend);
            return;
        case 4:
            cc.cdq(upper_half, dividend);
            return;
        case 8:
            cc.cqo(upper_half, dividend);
            return;
        default:
            throw std::runtime_error(fmt::format("Cannot prepare division for operand of size {}", dividend.size()));
        }
    }
    template <typename V, typename W, typename = std::enable_if_t<std::is_integral_v<V> && std::is_integral_v<W>>>
    x86_reg_t gen_operation(x86::Compiler& cc, complex_operation op, V a, W b) {
        x86_reg_t a_reg = get_reg(cc, sizeof(a) * 8);
        x86_reg_t b_reg = get_reg(cc, sizeof(b) * 8);
        mov(cc, a_reg, a);
        mov(cc, b_reg, b);
        return gen_operation(cc, op, a_reg, b_reg);
    }
    template <typename T, typename = std::enable_if_t<std::is_integral<T>::value>>
    x86_reg_t gen_operation(x86::Compiler& cc, complex_operation op, x86_reg_t a, T b) {
        auto size = 0;
        if(std::holds_alternative<x86::Gp>(a))
            size = std::get<x86::Gp>(a).size() * 8;
        else if(std::holds_alternative<dGp>(a))
            size = 128;
        else
            throw std::runtime_error("Invalid variant in gen_operation");
        x86_reg_t b_reg = get_reg(cc, size);
        mov(cc, b_reg, b);
        return gen_operation(cc, op, a, b_reg);
    }
    x86_reg_t gen_operation(x86::Compiler& cc, complex_operation op, x86_reg_t _a, x86_reg_t _b) {
        if(std::holds_alternative<x86::Gp>(_a) && std::holds_alternative<x86::Gp>(_b)) {
            x86::Gp a = std::get<x86::Gp>(_a);
            x86::Gp b = std::get<x86::Gp>(_b);
            assert(a.size() == b.size());
            x86::Gp overflow = get_reg_Gp(cc, a.size() * 8);
            switch(op) {
            case smul: {
                return _multiply(cc, op, a, b);
            }
            case umul: {
                return _multiply(cc, op, a, b);
            }
            case sumul: {
                if(a.size() <= 4) {
                    // As there is no mixed multiplication in x86, sign extend the signed value and zero extend the unsigned value
                    // then we can use normal signed multiplication
                    x86::Gp big_a = gen_ext_Gp(cc, a, a.size() * 8 * 2, true);
                    x86::Gp big_b = gen_ext_Gp(cc, b, b.size() * 8 * 2, false);
                    x86::Gp big_overflow = get_reg_Gp(cc, overflow.size() * 8 * 2, false); // This should always be empty
                    cc.imul(big_overflow, big_a, big_b);
                    return big_a;
                } else if(a.size() == 8) {
                    /*
                    The following algorithm was obtained by running godbolt on a function that looks like this:
                    __int128 multiply_signed_unsigned(int64_t num1, uint64_t num2) { return (__int128)num1 * (__int128)num2; }

                    Input rdi (num1) and rsi (num2)

                    mov     rax, rdi
                    cqo                 // sign extend rax into rdx:rax
                    mov     rcx, rdx
                    imul    rcx, rsi    // rcx * rsi -> rcx (truncate)
                    mul     rsi         // rsi * rax -> rdx:rax
                    add     rcx, rdx

                    Output in rcx:rax
                    */
                    x86::Gp rdi = a;
                    x86::Gp rsi = b;
                    x86::Gp rax = get_reg_Gp(cc, 64);
                    x86::Gp rdx = get_reg_Gp(cc, 64);
                    x86::Gp rcx = get_reg_Gp(cc, 64);
                    x86::Gp overflow = get_reg_Gp(cc, 64);
                    cc.mov(rax, rdi);
                    cc.cqo(rdx, rax);
                    cc.mov(rcx, rdx);
                    cc.imul(overflow, rcx, rsi);
                    cc.mul(rdx, rax, rsi);
                    cc.add(rcx, rdx);
                    dGp ret_val = get_reg_dGp(cc, 128);
                    ret_val.upper = rcx;
                    ret_val.lower = rax;
                    return ret_val;
                }
            }
            case sdiv: {
                expand_division_operand(cc, overflow, a);
                cc.idiv(overflow, a, b);
                return a;
            }
            case udiv: {
                cc.mov(overflow, 0);
                cc.div(overflow, a, b);
                return a;
            }
            case srem: {
                // division changes the contents of the a register, in this case as a side effect. Move it out of the way
                x86::Gp a_clone = get_reg_Gp(cc, a.size() * 8);
                cc.mov(a_clone, a);
                expand_division_operand(cc, overflow, a_clone);
                cc.idiv(overflow, a_clone, b);
                return overflow;
            }
            case urem: {
                x86::Gp a_clone = get_reg_Gp(cc, a.size() * 8);
                cc.mov(a_clone, a);
                cc.mov(overflow, 0);
                cc.div(overflow, a_clone, b);
                return overflow;
            }

            default:
                throw std::runtime_error(fmt::format("Current operation {} not supported in gen_operation (complex_operation)", op));
            }
        } else {
            throw std::runtime_error("Variant combination not supported in gen_operation (complex_operation)");
        }
    }
    x86_reg_t _multiply(x86::Compiler& cc, complex_operation op, x86_reg_t _a, x86_reg_t _b) {
        if(std::holds_alternative<x86::Gp>(_a) && std::holds_alternative<x86::Gp>(_b)) {
            x86::Gp a = std::get<x86::Gp>(_a);
            x86::Gp b = std::get<x86::Gp>(_b);
            x86::Gp overflow = get_reg_Gp(cc, a.size() * 8, false);
            // Multiplication of two XLEN wide registers returns a value that is 2*XLEN wide
            // do the multiplication and piece together the return register
            switch(op) {
            case smul: {
                cc.imul(overflow, a, b);
                break;
            }
            case umul: {
                cc.mul(overflow, a, b);
                break;
            }
            default:
                throw std::runtime_error(fmt::format("Current operation {} not supported in _multiply", op));
            }
            if(a.size() <= 4) {
                // Still return a single Gp
                // Obtain regs twice as big so that we can shift for the entire length
                x86::Gp big_a = gen_ext_Gp(cc, a, a.size() * 8 * 2, false);
                x86::Gp big_o = gen_ext_Gp(cc, overflow, overflow.size() * 8 * 2, false);
                cc.shl(big_o, overflow.size() * 8);
                cc.or_(big_a, big_o);
                return big_a;
            } else if(a.size() == 8) {
                dGp ret_val = get_reg_dGp(cc, 128);
                ret_val.upper = overflow;
                ret_val.lower = a;
                return ret_val;
            }
        } else if(std::holds_alternative<dGp>(_a) && std::holds_alternative<dGp>(_b)) {
            throw std::runtime_error("Multiplication with input operands >64-bit not yet implemented");
            return _b;
        }
        throw std::runtime_error("Variant combination not handled in _gen_operation (complex)");
        return _b;
    }
    template <typename T, typename = std::enable_if_t<std::is_integral<T>::value>>
    x86_reg_t gen_operation(x86::Compiler& cc, comparison_operation op, x86_reg_t _a, T b) {
        if(std::holds_alternative<x86::Gp>(_a)) {
            x86::Gp a = std::get<x86::Gp>(_a);
            return _gen_operation(cc, op, a, b);
        } else {
            throw std::runtime_error("Variant not supported in gen_operation (comparison)");
        }
    }

    x86_reg_t gen_operation(x86::Compiler& cc, comparison_operation op, x86_reg_t _a, x86_reg_t _b) {
        if(std::holds_alternative<x86::Gp>(_a) && std::holds_alternative<x86::Gp>(_b)) {
            x86::Gp a = std::get<x86::Gp>(_a);
            x86::Gp b = std::get<x86::Gp>(_b);
            return _gen_operation(cc, op, a, b);
        } else {
            throw std::runtime_error("Variant combination not supported in gen_operation (comparison)");
        }
    }
    x86::Gp _gen_operation(x86::Compiler& cc, comparison_operation op, x86::Gp a, x86::Gp b) {
        size_t a_size = a.size();
        size_t b_size = b.size();
        assert(a.size() == b.size());
        x86::Gp tmp = cc.newInt8();
        cc.mov(tmp, 1);
        Label label_then = cc.newLabel();
        cmp(cc, a, b);
        switch(op) {
        case eq:
            cc.je(label_then);
            break;
        case ne:
            cc.jne(label_then);
            break;
        case lt:
            cc.jl(label_then);
            break;
        case ltu:
            cc.jb(label_then);
            break;
        case gt:
            cc.jg(label_then);
            break;
        case gtu:
            cc.ja(label_then);
            break;
        case lte:
            cc.jle(label_then);
            break;
        case lteu:
            cc.jbe(label_then);
            break;
        case gte:
            cc.jge(label_then);
            break;
        case gteu:
            cc.jae(label_then);
            break;
        case land: {
            Label label_false = cc.newLabel();
            cc.cmp(a, 0);
            cc.je(label_false);
            auto b_reg = cc.newInt8();
            cc.mov(b_reg, b);
            cc.cmp(b_reg, 0);
            cc.je(label_false);
            cc.jmp(label_then);
            cc.bind(label_false);
            break;
        }
        case lor: {
            cc.cmp(a, 0);
            cc.jne(label_then);
            auto b_reg = cc.newInt8();
            cc.mov(b_reg, b);
            cc.cmp(b_reg, 0);
            cc.jne(label_then);
            break;
        }
        default:
            throw std::runtime_error(fmt::format("Current operation {} not supported in gen_operation (comparison)", op));
        }
        cc.mov(tmp, 0);
        cc.bind(label_then);
        return tmp;
    }
    template <typename T, typename = std::enable_if_t<std::is_integral_v<T> || std::is_same<T, x86::Gp>::value>>
    x86::Gp _gen_operation(x86::Compiler& cc, comparison_operation op, x86::Gp a, T b) {
        x86::Gp b_reg = get_reg_Gp(cc, sizeof(T) * 8);
        cc.mov(b_reg, b);
        x86::Gp big_b = gen_ext_Gp(cc, b_reg, a.size() * 8, std::is_signed_v<T>);
        return _gen_operation(cc, op, a, big_b);
    }
    x86_reg_t gen_operation(x86::Compiler& cc, unary_operation op, x86_reg_t _a) {
        if(std::holds_alternative<x86::Gp>(_a)) {
            x86::Gp a = std::get<x86::Gp>(_a);
            switch(op) {
            case lnot:
                throw std::runtime_error("Current operation not supported in gen_operation(lnot)");
            case inc: {
                cc.inc(a);
                break;
            }
            case dec: {
                cc.dec(a);
                break;
            }
            case bnot: {
                cc.not_(a);
                break;
            }
            case neg: {
                cc.neg(a);
                break;
            }
            default:
                throw std::runtime_error(fmt::format("Current operation {} not supported in gen_operation (unary)", op));
            }
            return a;
        } else {
            throw std::runtime_error("Variant not supported in gen_operation (unary)");
        }
    }
};
} // namespace asmjit
} // namespace iss
#endif /* ASMJIT_VM_BASE_H_ */
