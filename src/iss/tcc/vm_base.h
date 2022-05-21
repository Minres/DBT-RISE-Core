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

#ifndef _VM_BASE_H_
#define _VM_BASE_H_

#include "jit_helper.h"
#include <iss/arch/traits.h>
#include <iss/arch_if.h>
#include <iss/debugger/target_adapter_base.h>
#include <iss/debugger_if.h>
#include <iss/tcc/code_builder.h>
#include <util/ities.h>
#include <util/range_lut.h>
#include <iss/vm_if.h>
#include <iss/vm_plugin.h>
#include <util/logging.h>

#include <array>
#include <chrono>
#include <map>
#include <sstream>
#include <stack>
#include <utility>
#include <vector>

namespace iss {

namespace tcc {

enum continuation_e { CONT, BRANCH, FLUSH, TRAP };

template <typename ARCH> class vm_base : public debugger_if, public vm_if {
    struct plugin_entry {
        sync_type sync;
        vm_plugin &plugin;
        void *plugin_ptr; //FIXME: hack
    };
public:
    using reg_e = typename arch::traits<ARCH>::reg_e;
    using sr_flag_e = typename arch::traits<ARCH>::sreg_flag_e;
    using virt_addr_t = typename arch::traits<ARCH>::virt_addr_t;
    using phys_addr_t = typename arch::traits<ARCH>::phys_addr_t;
    using addr_t = typename arch::traits<ARCH>::addr_t;
    using code_word_t = typename arch::traits<ARCH>::code_word_t;
    using mem_type_e = typename arch::traits<ARCH>::mem_type_e;
    using tu_builder = typename iss::tcc::code_builder<ARCH>;
    using dbg_if = iss::debugger_if;

    constexpr static unsigned blk_size = 128; // std::numeric_limits<unsigned>::max();

    arch_if *get_arch() override { return &core; };

    constexpr unsigned int get_reg_width(int idx) const {
        return idx < 0 ? arch::traits<ARCH>::NUM_REGS : arch::traits<ARCH>::reg_bit_widths[(reg_e)idx];
    }

    template <typename T> inline T get_reg(unsigned r) {
        std::vector<uint8_t> res(sizeof(T), 0);
        uint8_t *reg_base = core.get_regs_base_ptr() + arch::traits<ARCH>::reg_byte_offsets[r];
        auto size = arch::traits<ARCH>::reg_bit_widths[r] / 8;
        std::copy(reg_base, reg_base + size, res.data());
        return *reinterpret_cast<T *>(&res[0]);
    }

    using func_ptr = uint64_t (*)(uint8_t *, void *, void *);

    int start(uint64_t icount = std::numeric_limits<uint64_t>::max(), bool dump = false,
            finish_cond_e cond = finish_cond_e::COUNT_LIMIT | finish_cond_e::JUMP_TO_SELF) override {
        int error = 0;
        if (this->debugging_enabled()) sync_exec = PRE_SYNC;
        auto start = std::chrono::high_resolution_clock::now();
        virt_addr_t pc(iss::access_type::DEBUG_FETCH, 0,
                       get_reg<typename arch::traits<ARCH>::addr_t>(arch::traits<ARCH>::PC));
        LOG(INFO) << "Start at 0x" << std::hex << pc.val << std::dec;
        try {
            continuation_e cont = CONT;
            // struct to minimize the type size of the closure below to allow SSO
            struct {
                vm_base *vm;
                virt_addr_t &pc;
                continuation_e &cont;
            } param = {this, pc, cont};
            //translation_block getPointerToFunction(unsigned cluster_id, uint64_t phys_addr, gen_func &generator, bool dumpEnabled);

            iss::tcc::gen_func generator{[&param]() -> std::tuple<std::string, std::string> {
                std::string fname;
                std::string code;
                std::tie(param.cont, fname, code) = param.vm->disass(param.pc);
                param.vm->mod = nullptr;
                param.vm->func = nullptr;
                return std::make_tuple(fname, code);
            }};
            // explicit std::function to allow use as reference in call below
            // std::function<Function*(Module*)> gen_ref(std::ref(generator));
            translation_block *last_tb = nullptr, *cur_tb = nullptr;
            uint32_t last_branch = std::numeric_limits<uint32_t>::max();
            arch_if *const arch_if_ptr = static_cast<arch_if *>(&core);
            vm_if *const vm_if_ptr = static_cast<vm_if *>(this);
            while (!core.should_stop() && core.get_icount() < icount) {
                try {
                    // translate into physical address
                    const auto pc_p = core.v2p(pc);
                    // check if we have the block already compiled
                    auto it = this->func_map.find(pc_p.val);
                    if (it == this->func_map.end()) { // if not generate and compile it
                        auto res = func_map.insert(std::make_pair(
                            pc_p.val, getPointerToFunction(cluster_id, pc_p.val, generator, dump)));
                        it = res.first;
                    }
                    cur_tb = &(it->second);
                    // if we have a previous block link the just compiled one as successor of the last tb
                    if (last_tb && last_branch < 2 && last_tb->cont[last_branch] == nullptr)
                        last_tb->cont[last_branch] = cur_tb;
                    do {
                        // execute the compiled function
                        pc.val = reinterpret_cast<func_ptr>(cur_tb->f_ptr)(regs_base_ptr, arch_if_ptr, vm_if_ptr);
                        // update last state
                        last_tb = cur_tb;
                        last_branch = core.get_last_branch();
                        auto cur_icount = core.get_icount();
                        // if the current tb has a successor assign to current tb
                        if (last_branch < 2 && cur_tb->cont[last_branch] != nullptr && cur_icount < icount)
                            cur_tb = cur_tb->cont[last_branch];
                        else // if not we need to compile one
                            cur_tb = nullptr;
                    } while (cur_tb != nullptr);
                    if (cont == FLUSH) {
                        //for (auto &e : func_map) delete (e.second.mod_eng);
                        func_map.clear();
                    }
                    if (cont == TRAP) {
                        auto it = func_map.find(pc_p.val);
                        if (it != func_map.end()) {
                            func_map.erase(it);
                        }
                    }
                } catch (trap_access &ta) {
                    pc.val = core.enter_trap(ta.id, ta.addr, 0);
                }
#ifndef NDEBUG
                LOG(TRACE) << "continuing  @0x" << std::hex << pc << std::dec;
#endif
            }
            LOG(INFO) << "ISS execution finished";
            error = core.stop_code() > 1? core.stop_code():0;
       } catch (simulation_stopped &e) {
            LOG(INFO) << "ISS execution stopped with status 0x" << std::hex << e.state << std::dec;
            if (e.state != 1) error = e.state;
        } catch (decoding_error &e) {
            LOG(ERR) << "ISS execution aborted at address 0x" << std::hex << e.addr << std::dec;
            error = -1;
        }
        auto end = std::chrono::high_resolution_clock::now(); // end measurement
                                                              // here
        auto elapsed = end - start;
        auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
        LOG(INFO) << "Executed " << core.get_icount() << " instructions in " << func_map.size()
                  << " code blocks during " << millis << "ms resulting in " << (core.get_icount() * 0.001 / millis)
                  << "MIPS";
        return error;
    }

    void reset() override { core.reset(); }

    void reset(uint64_t address) { core.reset(address); }

    void pre_instr_sync() override {
        uint64_t pc = get_reg<typename arch::traits<ARCH>::addr_t>(arch::traits<ARCH>::PC);
        tgt_adapter->check_continue(pc);
    }

protected:
    std::tuple<continuation_e, std::string, std::string> disass(virt_addr_t &pc) {
        unsigned cur_blk = 0;
        virt_addr_t cur_pc = pc;
        std::pair<virt_addr_t, phys_addr_t> cur_pc_mark(pc, this->core.v2p(pc));
        unsigned int num_inst = 0;
        tu_builder tu;
        open_block_func(tu, cur_pc_mark.second);
        continuation_e cont = CONT;
        try {
            while (cont == CONT && cur_blk < blk_size) {
                std::tie(cont) = gen_single_inst_behavior(cur_pc, num_inst, tu);
                cur_blk++;
            }
            const phys_addr_t end_pc(this->core.v2p(--cur_pc));
            assert(cur_pc_mark.first.val <= cur_pc.val);
            close_block_func(tu);
            return std::make_tuple(cont, tu.fname, tu.finish());
        } catch (trap_access &ta) {
            const phys_addr_t end_pc(this->core.v2p(--cur_pc));
            if (cur_pc_mark.first.val <= cur_pc.val) { // close the block and return result up to here
                close_block_func(tu);
               return std::make_tuple(cont, tu.fname, tu.finish());
            } else // re-throw if it was the first instruction
                throw ta;
        }
    }

    virtual void setup_module(std::string m) {}

    virtual std::tuple<continuation_e>
    gen_single_inst_behavior(virt_addr_t &pc_v, unsigned int &inst_cnt, tu_builder& tu) = 0;

    virtual void gen_trap_behavior(tu_builder& tu) = 0;

    explicit vm_base(ARCH &core, unsigned core_id = 0, unsigned cluster_id = 0)
    : core(core)
    , core_id(core_id)
    , cluster_id(cluster_id)
    , regs_base_ptr(core.get_regs_base_ptr())
    , sync_exec(NO_SYNC)
    , mod(nullptr)
    , func(nullptr)
    , tgt_adapter(nullptr) {
        sync_exec = static_cast<sync_type>(sync_exec | core.needed_sync());
    }

    ~vm_base() override { delete tgt_adapter; }

    void register_plugin(vm_plugin &plugin) {
        if (plugin.registration("1.0", *this)) {
            plugins.push_back(plugin_entry{plugin.get_sync(), plugin, &plugin});
        }
    }

    inline void *get_reg_ptr(unsigned i) {
        return regs_base_ptr + arch::traits<ARCH>::reg_byte_offsets[i];
    }

    // NO_SYNC = 0, PRE_SYNC = 1, POST_SYNC = 2, ALL_SYNC = 3
    const std::array<const iss::arch_if::exec_phase, 4> notifier_mapping = {
        {iss::arch_if::ISTART, iss::arch_if::ISTART, iss::arch_if::IEND, iss::arch_if::ISTART}};

    inline void gen_sync(tu_builder& tu, sync_type s, unsigned inst_id) {
        if (s == PRE_SYNC) {
            // update icount
            tu("(*icount)++;");
            tu("*pc=*next_pc;");
            tu("*trap_state=*pending_trap;");
            if (debugging_enabled())
                tu("pre_instr_sync(vm_ptr);");
        }
        if ((s & sync_exec))
            tu("notify_phase(core_ptr, {});", notifier_mapping[s]);
        iss::instr_info_t iinfo{cluster_id, core_id, inst_id, s};
        for (plugin_entry e : plugins) {
            if (e.sync & s)
                tu("call_plugin((void*){}, (uint64_t){})", e.plugin_ptr, iinfo.backing.val);
        }
    }

    void open_block_func(tu_builder& tu, phys_addr_t pc) {
        tu.fname = fmt::format("tcc_jit_{:#x}", pc.val);
    }

    void close_block_func(tu_builder& tu){
        tu("return *next_pc;");
        gen_trap_behavior(tu);
    }

    ARCH &core;
    unsigned core_id = 0;
    unsigned cluster_id = 0;
    uint8_t *regs_base_ptr;
    sync_type sync_exec;
    std::unordered_map<uint64_t, translation_block> func_map;
    // non-owning pointers
    void *mod;
    void *func;
    // std::vector<Value *> loaded_regs{arch::traits<ARCH>::NUM_REGS, nullptr};
    iss::debugger::target_adapter_base *tgt_adapter;
    std::vector<plugin_entry> plugins;
};
}
}

#endif /* _VM_BASE_H_ */
