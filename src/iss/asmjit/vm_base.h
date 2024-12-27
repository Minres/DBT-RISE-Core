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
#include <iss/asmjit/vm_util.h>
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

enum continuation_e { CONT, BRANCH, FLUSH, TRAP, ILLEGAL_INSTR, JUMP_TO_SELF, ILLEGAL_FETCH };
enum last_branch_e { NO_JUMP = 0, KNOWN_JUMP = 1, UNKNOWN_JUMP = 2, BRANCH_TO_SELF = 3 };

template <typename ARCH> class vm_base : public debugger_if, public vm_if {
    struct plugin_entry {
        sync_type sync;
        vm_plugin& plugin;
        void* plugin_ptr; // FIXME: hack
    };

public:
    using reg_e = typename arch::traits<ARCH>::reg_e;
    using reg_t = typename arch::traits<ARCH>::reg_t;
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

    template <typename T> inline T obtain_reg(unsigned r) {
        std::vector<uint8_t> res(sizeof(T), 0);
        uint8_t* reg_base = regs_base_ptr + arch::traits<ARCH>::reg_byte_offsets[r];
        auto size = arch::traits<ARCH>::reg_bit_widths[r] / 8;
        std::copy(reg_base, reg_base + size, res.data());
        return *reinterpret_cast<T*>(&res[0]);
    }

    template <typename T = reg_t> inline T& get_reg_ref(unsigned r) {
        return *reinterpret_cast<T*>(regs_base_ptr + arch::traits<ARCH>::reg_byte_offsets[r]);
    }

    using func_ptr = uint64_t (*)(uint8_t*, void*, void*);

    int start(uint64_t icount_limit = std::numeric_limits<uint64_t>::max(), bool dump = false,
              finish_cond_e cond = finish_cond_e::ICOUNT_LIMIT | finish_cond_e::JUMP_TO_SELF) override {
        int error = 0;
        uint32_t was_illegal = 0;
        if(this->debugging_enabled())
            sync_exec |= PRE_SYNC;
        auto start = std::chrono::high_resolution_clock::now();
        virt_addr_t pc(iss::access_type::DEBUG_FETCH, 0, obtain_reg<typename arch::traits<ARCH>::addr_t>(arch::traits<ARCH>::PC));
        CPPLOG(INFO) << "Start at 0x" << std::hex << pc.val << std::dec;
        try {
            continuation_e cont = CONT;

            std::function<void(jit_holder&)> generator{[this, &pc, &cont, icount_limit](jit_holder& jh) -> void {
                gen_block_prologue(jh);
                cont = translate(pc, jh, icount_limit);
                gen_block_epilogue(jh);
                // move local disass collection to global collection by appending
                this->global_disass_collection.insert(this->global_disass_collection.end(), jh.disass_collection.begin(),
                                                      jh.disass_collection.end());
                jh.disass_collection.clear();
            }};
            // explicit std::function to allow use as reference in call below
            // std::function<Function*(Module*)> gen_ref(std::ref(generator));
            translation_block *last_tb = nullptr, *cur_tb = nullptr;
            auto& last_branch = get_reg_ref(reg_e::LAST_BRANCH);
            uint64_t& cur_icount = get_reg_ref<uint64_t>(reg_e::ICOUNT);
            arch_if* const arch_if_ptr = static_cast<arch_if*>(&core);
            vm_if* const vm_if_ptr = static_cast<vm_if*>(this);
            while(!core.should_stop() && cur_icount < icount_limit) {
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
                    if(cont == JUMP_TO_SELF || last_branch == BRANCH_TO_SELF)
                        throw simulation_stopped(0);
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
                        // if the current tb has a successor assign to current tb
                        if(last_branch < 2 && cur_tb->cont[last_branch] != nullptr && cur_icount < icount_limit) {
                            cur_tb = cur_tb->cont[last_branch];
                            // update cont, as it only gets set when a new fptr gets created
                            cont = static_cast<continuation_e>(last_branch);
                        } else // if not we need to compile one
                            cur_tb = nullptr;
                    } while(cur_tb != nullptr);
                    if(cont == FLUSH)
                        func_map.clear();
                    if(cont == ILLEGAL_INSTR) {
                        if(was_illegal > 2) {
                            CPPLOG(ERR) << "ISS execution aborted after trying to execute illegal instructions 3 times in a row";
                            error = -1;
                            break;
                        }
                        was_illegal++;
                    } else {
                        was_illegal = 0;
                    }
                } catch(trap_access& ta) {
                    pc.val = core.enter_trap(1 << 16, ta.addr, 0);
                }
#ifndef NDEBUG
                CPPLOG(TRACE) << "continuing  @0x" << std::hex << pc << std::dec;
#endif
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
        auto elapsed = end - start;
        auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
        auto cur_icount = get_reg_ref<uint64_t>(arch::traits<ARCH>::reg_e::ICOUNT);
        CPPLOG(INFO) << "Executed " << cur_icount << " instructions in " << func_map.size() << " code blocks during " << millis
                     << "ms resulting in " << (cur_icount * 0.001 / millis) << "MIPS";
        return error;
    }

    void reset() override { core.reset(); }

    void reset(uint64_t address) override { core.reset(address); }

    void pre_instr_sync() override {
        uint64_t pc = obtain_reg<typename arch::traits<ARCH>::addr_t>(arch::traits<ARCH>::PC);
        tgt_adapter->check_continue(pc);
    }

protected:
    continuation_e translate(virt_addr_t pc, jit_holder& jh, uint64_t icount_limit) {
        unsigned cur_blk_size = 0;
        continuation_e cont = CONT;
        while(cont == CONT && cur_blk_size < blk_size && cur_blk_size < icount_limit) {
            cont = gen_single_inst_behavior(pc, jh);
            cur_blk_size++;
        }
        if(cont == ILLEGAL_FETCH && cur_blk_size == 1) {
            throw trap_access(0, pc.val);
        }
        return cont;
    }
    virtual continuation_e gen_single_inst_behavior(virt_addr_t&, jit_holder&) = 0;
    virtual void gen_block_prologue(jit_holder&) = 0;
    virtual void gen_block_epilogue(jit_holder&) = 0;

    explicit vm_base(ARCH& core, unsigned core_id = 0, unsigned cluster_id = 0)
    : core(core)
    , core_id(core_id)
    , cluster_id(cluster_id)
    , regs_base_ptr(core.get_regs_base_ptr()) {
        sync_exec = static_cast<sync_type>(sync_exec | core.needed_sync());
    }

    explicit vm_base(std::unique_ptr<ARCH> core_ptr, unsigned core_id = 0, unsigned cluster_id = 0)
    : core(*core_ptr)
    , core_ptr(std::move(core_ptr))
    , core_id(core_id)
    , cluster_id(cluster_id)
    , regs_base_ptr(core.get_regs_base_ptr()) {
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

    ARCH& core;
    std::unique_ptr<ARCH> core_ptr;
    unsigned core_id = 0;
    unsigned cluster_id = 0;
    uint8_t* regs_base_ptr{nullptr};
    sync_type sync_exec{sync_type::NO_SYNC};
    std::unordered_map<uint64_t, translation_block> func_map;
    iss::debugger::target_adapter_base* tgt_adapter{nullptr};
    std::vector<plugin_entry> plugins;
    std::vector<char*> global_disass_collection;

    // Asmjit generator functions

    void gen_leave(jit_holder& jh, unsigned lvl) {
        InvokeNode* call_leave;
        jh.cc.comment("//gen_leave");
        jh.cc.invoke(&call_leave, &leave_trap, FuncSignature::build<void, void*, uint64_t>());
        call_leave->setArg(0, this->get_arch());
        call_leave->setArg(1, lvl);
        mov(jh.cc, get_ptr_for(jh, traits::LAST_BRANCH), static_cast<int>(UNKNOWN_JUMP));
        jh.next_pc = load_reg_from_mem_Gp(jh, traits::NEXT_PC);
    }
    void write_back(jit_holder& jh) {
        write_reg_to_mem(jh, jh.pc, traits::PC);
        write_reg_to_mem(jh, jh.next_pc, traits::NEXT_PC);
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
        if(nonstd::holds_alternative<x86::Gp>(return_reg)) {
            return nonstd::get<x86::Gp>(return_reg);
        } else {
            throw std::runtime_error("Invalid variation returned in load_reg_from_mem_Gp");
        }
    }
    inline void write_reg_to_mem(jit_holder& jh, x86_reg_t reg, unsigned idx) {
        if(nonstd::holds_alternative<dGp>(reg)) {
            throw std::runtime_error("Writing dGp to memory not implemented");
        }
        auto ptr = get_ptr_for(jh, idx);
        mov(jh.cc, ptr, reg);
    }

    inline x86_reg_t gen_read_mem(jit_holder& jh, mem_type_e type, x86_reg_t _addr, uint32_t length) {
        if(nonstd::holds_alternative<x86::Gp>(_addr)) {
            auto addr = nonstd::get<x86::Gp>(_addr);
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
        else if(nonstd::holds_alternative<dGp>(_addr)) {
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
        if(nonstd::holds_alternative<x86::Gp>(_addr) && nonstd::holds_alternative<x86::Gp>(_val)) {
            auto addr = nonstd::get<x86::Gp>(_addr);
            auto val = nonstd::get<x86::Gp>(_val);
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
};
} // namespace asmjit
} // namespace iss
#endif /* ASMJIT_VM_BASE_H_ */
