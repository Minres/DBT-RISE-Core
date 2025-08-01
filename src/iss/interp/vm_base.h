/*******************************************************************************
 * Copyright (C) 2017 - 2023, MINRES Technologies GmbH
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

#include <iss/arch/traits.h>
#include <iss/arch_if.h>
#include <iss/debugger/target_adapter_base.h>
#include <iss/debugger_if.h>
#include <iss/vm_if.h>
#include <iss/vm_plugin.h>
#include <util/ities.h>
#include <util/logging.h>
#include <util/range_lut.h>

#include <array>
#include <chrono>
#include <map>
#include <sstream>
#include <stack>
#include <type_traits>
#include <utility>
#include <vector>

#ifndef _MSC_VER
using int128_t = __int128;
using uint128_t = unsigned __int128;
namespace std {
template <> struct make_unsigned<__int128> { typedef unsigned __int128 type; };
template <> class __make_unsigned_selector<__int128 unsigned, false, false> {
public:
    typedef unsigned __int128 __type;
};
template <> struct is_signed<int128_t> { static constexpr bool value = true; };
template <> struct is_signed<uint128_t> { static constexpr bool value = false; };
template <> struct is_unsigned<int128_t> { static constexpr bool value = false; };
template <> struct is_unsigned<uint128_t> { static constexpr bool value = true; };
} // namespace std
#endif

namespace iss {

namespace interp {

enum continuation_e { CONT, BRANCH, FLUSH, TRAP };

template <typename ARCH> class vm_base : public debugger_if, public vm_if {
    struct plugin_entry {
        vm_plugin& plugin;
    };

public:
    using reg_e = typename arch::traits<ARCH>::reg_e;
    using sr_flag_e = typename arch::traits<ARCH>::sreg_flag_e;
    using virt_addr_t = typename arch::traits<ARCH>::virt_addr_t;
    using phys_addr_t = typename arch::traits<ARCH>::phys_addr_t;
    using addr_t = typename arch::traits<ARCH>::addr_t;
    using reg_t = typename arch::traits<ARCH>::reg_t;
    using code_word_t = typename arch::traits<ARCH>::code_word_t;
    using mem_type_e = typename arch::traits<ARCH>::mem_type_e;

    using dbg_if = iss::debugger_if;

    constexpr static unsigned blk_size = 128; // std::numeric_limits<unsigned>::max();

    arch_if* get_arch() override { return &core; };

    constexpr unsigned int get_reg_width(int idx) const {
        return idx < 0 ? arch::traits<ARCH>::NUM_REGS : arch::traits<ARCH>::reg_bit_widths[(reg_e)idx];
    }

    template <typename T> inline T get_reg_val(unsigned r) {
        std::array<uint8_t, sizeof(T)> res;
        uint8_t* reg_base = regs_base_ptr + arch::traits<ARCH>::reg_byte_offsets[r];
        auto size = arch::traits<ARCH>::reg_bit_widths[r] / 8;
        std::copy(reg_base, reg_base + size, res.data());
        return *reinterpret_cast<T*>(&res[0]);
    }

    template <typename T = reg_t> inline T& get_reg(unsigned r) {
        return *reinterpret_cast<T*>(regs_base_ptr + arch::traits<ARCH>::reg_byte_offsets[r]);
    }
    int start(uint64_t count = std::numeric_limits<uint64_t>::max(), bool dump = false,
              finish_cond_e cond = finish_cond_e::ICOUNT_LIMIT | finish_cond_e::JUMP_TO_SELF) override {
        int error = 0;
        auto start = std::chrono::high_resolution_clock::now();
        virt_addr_t pc(iss::access_type::FETCH, arch::traits<ARCH>::MEM, get_reg<addr_t>(arch::traits<ARCH>::PC));
        if(this->debugging_enabled()) {
            CPPLOG(INFO) << "Start at 0x" << std::hex << pc.val << std::dec << ", waiting for debugger";
        } else
            CPPLOG(INFO) << "Start at 0x" << std::hex << pc.val << std::dec;
        try {
            execute_inst(cond, pc, count);
        } catch(simulation_stopped& e) {
            CPPLOG(INFO) << "ISS execution stopped with status 0x" << std::hex << e.state << std::dec;
            if(e.state != 1)
                error = e.state;
            this->core.interrupt_sim=true;
        }
        auto end = std::chrono::high_resolution_clock::now(); // end measurement
                                                              // here
        auto elapsed = end - start;
        auto millis = std::chrono::duration_cast<std::chrono::milliseconds>(elapsed).count();
        auto instr_if = core.get_instrumentation_if();
        if(instr_if) {
            CPPLOG(INFO) << "Executed " << instr_if->get_instr_count() << " instructions in " << instr_if->get_total_cycles()
                         << " cycles during " << millis << "ms resulting in " << (instr_if->get_instr_count() * 0.001 / millis) << "MIPS";
        } else
            CPPLOG(INFO) << "Execution took " << millis << "ms";
        return error;
    }

    void reset() override { core.reset(); }

    void reset(uint64_t address) override { core.reset(address); }

    void pre_instr_sync() override {
        uint64_t pc = get_reg<typename arch::traits<ARCH>::addr_t>(arch::traits<ARCH>::PC);
        tgt_adapter->check_continue(pc);
    }

protected:
    virtual virt_addr_t execute_inst(finish_cond_e cond, virt_addr_t start, uint64_t count_limit) = 0;

    explicit vm_base(ARCH& core, unsigned core_id = 0, unsigned cluster_id = 0)
    : core(core)
    , core_id(core_id)
    , cluster_id(cluster_id)
    , regs_base_ptr(core.get_regs_base_ptr()) {
        init();
    }

    explicit vm_base(std::unique_ptr<ARCH> core_ptr, unsigned core_id = 0, unsigned cluster_id = 0)
    : core(*core_ptr)
    , core_ptr(std::move(core_ptr))
    , core_id(core_id)
    , cluster_id(cluster_id)
    , regs_base_ptr(core.get_regs_base_ptr()) {
        init();
    }

    ~vm_base() override { delete tgt_adapter; }

    void register_plugin(vm_plugin& plugin) override {
        if(plugin.registration("1.0", *this)) {
            auto sync = plugin.get_sync();
            if(sync & PRE_SYNC)
                pre_plugins.push_back(plugin_entry{plugin});
            if(sync & POST_SYNC)
                post_plugins.push_back(plugin_entry{plugin});
            sync_exec |= sync;
        }
    }

    // NO_SYNC = 0, PRE_SYNC = 1, POST_SYNC = 2, ALL_SYNC = 3
    const std::array<const iss::arch_if::exec_phase, 4> notifier_mapping = {
        {iss::arch_if::ISTART, iss::arch_if::ISTART, iss::arch_if::IEND, iss::arch_if::ISTART}};

    inline void do_sync(sync_type s, unsigned inst_id) {
        if((s & core_sync))
            core.notify_phase(notifier_mapping[s]);
        iss::instr_info_t iinfo{cluster_id, core_id, inst_id, static_cast<unsigned>(s)};
        if(s & PRE_SYNC)
            for(plugin_entry e : pre_plugins)
                e.plugin.callback(iinfo);
        else
            for(plugin_entry e : post_plugins)
                e.plugin.callback(iinfo);
    }

    template <typename DT, typename AT> inline DT read_mem(mem_type_e type, AT addr) {
        DT val;
        this->core.read(iss::address_type::VIRTUAL, access_type::READ, type,
                        static_cast<uint64_t>(static_cast<typename std::make_unsigned<AT>::type>(addr)), sizeof(DT),
                        reinterpret_cast<uint8_t*>(&val));
        return val;
    }

    template <typename DT, typename AT> inline void write_mem(mem_type_e type, AT addr, DT val) {
        this->core.write(iss::address_type::VIRTUAL, access_type::WRITE, type,
                         static_cast<uint64_t>(static_cast<typename std::make_unsigned<AT>::type>(addr)), sizeof(DT),
                         reinterpret_cast<uint8_t*>(&val));
    }

    template <typename TT, typename ST> inline TT sext(ST val) {
        return static_cast<TT>(static_cast<typename std::make_signed<ST>::type>(val));
    }

    template <typename TT, typename ST> inline TT zext(ST val) {
        return static_cast<TT>(static_cast<typename std::make_unsigned<ST>::type>(val));
    }

    template <typename TT, typename ST> inline TT trunc(ST val) {
        return static_cast<TT>(static_cast<typename std::make_unsigned<ST>::type>(val));
    }

    ARCH& core;
    std::unique_ptr<ARCH> core_ptr;
    unsigned core_id = 0;
    unsigned cluster_id = 0;
    uint8_t* regs_base_ptr;
    sync_type sync_exec{NO_SYNC};
    sync_type core_sync{NO_SYNC};
    // non-owning pointers
    // std::vector<Value *> loaded_regs{arch::traits<ARCH>::NUM_REGS, nullptr};
    iss::debugger::target_adapter_base* tgt_adapter{nullptr};
    std::vector<plugin_entry> pre_plugins;
    std::vector<plugin_entry> post_plugins;

private:
    void init() {
        sync_exec = static_cast<sync_type>(sync_exec | core.needed_sync());
        core_sync = static_cast<sync_type>(core_sync | core.needed_sync());
    }
};
} // namespace interp
} // namespace iss

#endif /* _VM_BASE_H_ */
