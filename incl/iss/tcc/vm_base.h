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

    int start(uint64_t icount = std::numeric_limits<uint64_t>::max(), bool dump = false) override {
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
                    pc.val = core.enter_trap(ta.id, ta.addr);
                }
#ifndef NDEBUG
                LOG(TRACE) << "continuing  @0x" << std::hex << pc << std::dec;
#endif
            }
        } catch (simulation_stopped &e) {
            LOG(INFO) << "ISS execution stopped with status 0x" << std::hex << e.state << std::dec;
            if (e.state != 1) error = e.state;
        } catch (decoding_error &e) {
            LOG(ERROR) << "ISS execution aborted at address 0x" << std::hex << e.addr << std::dec;
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
    enum class ICmpInst {ICMP_UGT, ICMP_ULT, ICMP_UGE, ICMP_ULE, ICMP_EQ, ICMP_NE, ICMP_SGT, ICMP_SLT, ICMP_SGE, ICMP_SLE};
    struct translation_unit {
        template <typename S, typename... Args>
        inline void operator()(const S& format_str, Args&&... args) {
            lines.push_back(fmt::format(format_str, args...));
        }
        void operator<<(std::string const& s){ lines.push_back(s);}
        void operator<<(std::string && s){ lines.push_back(s);}
        std::string fname;
        std::vector<std::string> lines{};
        std::array<bool, arch::traits<ARCH>::NEXT_PC+5> defined_regs{false};
        inline std::string add_reg_ptr(std::string const& name, unsigned reg_num){
            return fmt::format("  uint{0}_t* {2} = (uint{0}_t*)(regs_ptr+{1:#x});\n",
                    arch::traits<ARCH>::reg_bit_widths[reg_num],
                    arch::traits<ARCH>::reg_byte_offsets[reg_num], name);
        }

        std::string finish(){
            std::ostringstream os;
            // generate prologue
            os<<"#include <stdint.h>\n";
            os<<"#include <stdbool.h>\n";
            os<<"extern uint8_t read_mem1(void*, uint32_t, uint32_t, uint64_t);\n";
            os<<"extern void write_mem1(void*, uint32_t, uint32_t, uint64_t, uint8_t);\n";
            os<<"extern uint16_t read_mem2(void*, uint32_t, uint32_t, uint64_t);\n";
            os<<"extern void write_mem2(void*, uint32_t, uint32_t, uint64_t, uint16_t);\n";
            os<<"extern uint32_t read_mem4(void*, uint32_t, uint32_t, uint64_t);\n";
            os<<"extern void write_mem4(void*, uint32_t, uint32_t, uint64_t, uint32_t);\n";
            os<<"extern uint64_t read_mem8(void*, uint32_t, uint32_t, uint64_t);\n";
            os<<"extern void write_mem8(void*, uint32_t, uint32_t, uint64_t, uint64_t);\n";
            os<<"extern uint64_t enter_trap(void*, uint64_t, uint64_t);\n";
            os<<"extern uint64_t leave_trap(void*, uint64_t);\n";
            os<<"extern void wait(void*, uint64_t);\n";
            os<<"extern void print_string(void*, char*);\n";
            os<<"extern void print_disass(void*, uint64_t, char*);\n";
            os<<"extern void pre_instr_sync(void*);\n";
            os<<"extern void notify_phase(void*, uint32_t);\n";
            os<<"extern void call_plugin(void*, uint64_t) ;\n";
            //os<<fmt::format("typedef uint{}_t reg_t;\n", arch::traits<ARCH>::XLEN);
            os<<fmt::format("uint64_t {}(uint8_t* regs_ptr, void* core_ptr, void* vm_ptr) __attribute__ ((regnum(3)))  {{\n", fname);
            os<<add_reg_ptr("pc", arch::traits<ARCH>::PC);
            os<<add_reg_ptr("next_pc", arch::traits<ARCH>::NEXT_PC);
            os<<add_reg_ptr("trap_state", arch::traits<ARCH>::TRAP_STATE);
            os<<add_reg_ptr("pending_trap", arch::traits<ARCH>::PENDING_TRAP);
            os<<add_reg_ptr("icount", arch::traits<ARCH>::ICOUNT);

            for(size_t i=0; i<arch::traits<ARCH>::NUM_REGS; ++i){
                if(defined_regs[i]){
                    os<<fmt::format("  uint{0}_t* reg{2:02d} = (uint{0}_t*)(regs_ptr+{1:#x});\n",
                                    arch::traits<ARCH>::reg_bit_widths[i],
                                    arch::traits<ARCH>::reg_byte_offsets[i], i);
                }
            }
            if(defined_regs[arch::traits<ARCH>::LAST_BRANCH]){
                os<<fmt::format("  uint{0}_t* reg{2:02d} = (uint{0}_t*)(regs_ptr+{1:#x});\n",
                                arch::traits<ARCH>::reg_bit_widths[arch::traits<ARCH>::LAST_BRANCH],
                                arch::traits<ARCH>::reg_byte_offsets[arch::traits<ARCH>::LAST_BRANCH], arch::traits<ARCH>::LAST_BRANCH);
            }
            // add generated code
            std::copy(lines.begin(), lines.end(), std::ostream_iterator<std::string>(os, "\n"));
            // and the epilogue
            os<<"}";
            return os.str();
        }

        void open_scope() {
            lines.push_back("  {");
        }

        void close_scope() {
            lines.push_back("  }");
        }

        template <typename T, typename std::enable_if<std::is_signed<T>::value>::type * = nullptr>
        inline std::string gen_const(unsigned size, T val) const {
            return fmt::format("((int{}_t){})", size, val);
        }

        template <typename T, typename std::enable_if<!std::is_signed<T>::value>::type * = nullptr>
        inline std::string gen_const(unsigned size, T val) const {
            return fmt::format("((uint{}_t){})", size, val);
        }

        template <typename T>
        inline std::string gen_ext(T val, unsigned size, bool isSigned) const {
            if (isSigned)
                return fmt::format("((int{}_t){})", size, val);
            else
                return fmt::format("((uint{}_t){})", size, val);
        }

        inline std::string create_assignment(std::string const& name, std::string const& val, unsigned width){
            if(width==1){
                return fmt::format("  bool {} = {};", name, val);
            } else{
                return fmt::format("  uint{}_t {} = {};", width, name, val);
            }
        }

        inline std::string create_store(std::string const& val, unsigned reg_num){
            switch(reg_num){
            case arch::traits<ARCH>::NEXT_PC:
                return fmt::format("  *next_pc = {};", val);
            case arch::traits<ARCH>::PC:
                return fmt::format("  *pc = {};", val);
            default:
                defined_regs[reg_num]=true;
                return fmt::format("  *reg{:02d} = {};", reg_num, val);
            }
        }

        inline std::string create_load(unsigned reg_num, unsigned nesting_lvl){
            switch(reg_num){
            case arch::traits<ARCH>::NEXT_PC:
                return "(*next_pc)";
            case arch::traits<ARCH>::PC:
                return "(*pc)";
            default:
                defined_regs[reg_num]=true;
                return fmt::format("(*reg{:02d})", reg_num);
            }
        }

        inline std::string create_add(std::string const & left, std::string const & right){
            return fmt::format("({}) + ({})", left, right);
        }

        inline std::string create_sub(std::string const & left, std::string const & right){
            return fmt::format("({}) - ({})", left, right);
        }

        inline std::string create_and(std::string const & left, std::string const & right){
            return fmt::format("({}) & ({})", left, right);
        }

        inline std::string create_or(std::string const & left, std::string const & right){
            return fmt::format("({}) | ({})", left, right);
        }

        inline std::string create_xor(std::string const & left, std::string const & right){
            return fmt::format("({}) ^ ({})", left, right);
        }

        inline std::string create_b_and(std::string const & left, std::string const & right){
            return fmt::format("({}) && ({})", left, right);
        }

        inline std::string create_b_or(std::string const & left, std::string const & right){
            return fmt::format("({}) || ({})", left, right);
        }

        inline std::string create_not(std::string const & left){
            return fmt::format("~({})", left);
        }

        inline std::string create_neg(std::string const & left){
            return fmt::format("!({})", left);
        }

        inline std::string create_shl(std::string const & val,std::string const & shift){
            return fmt::format("({})<<({})", val, shift);
        }

        inline std::string create_lshr(std::string const & val,std::string const & shift){
            return fmt::format("((uint32_t){})<<({})", val, shift);
        }

        inline std::string create_ashr(std::string const & val,std::string const & shift){
            return fmt::format("((uint32_t){})<<({})", val, shift);
        }

        inline std::string create_choose(std::string const & cond, std::string const & left, std::string const & right){
            return fmt::format("({})?({}) : ({})", cond, left, right);
        }

        inline std::string create_icmp(ICmpInst inst, std::string const& left, std::string const& right){
            switch(inst){
            case ICmpInst::ICMP_SGT:
            case ICmpInst::ICMP_UGT:
                return fmt::format("{} > {}", left, right);
            case ICmpInst::ICMP_SLT:
            case ICmpInst::ICMP_ULT:
                return fmt::format("{} < {}", left, right);
            case ICmpInst::ICMP_SGE:
            case ICmpInst::ICMP_UGE:
                return fmt::format("{} >= {}", left, right);
            case ICmpInst::ICMP_SLE:
            case ICmpInst::ICMP_ULE:
                return fmt::format("{} <= {}", left, right);
            case ICmpInst::ICMP_EQ: return fmt::format("{} == {}", left, right);
            case ICmpInst::ICMP_NE: return fmt::format("{} != {}", left, right);
            }
            return "";
        }

        inline std::string create_zext_or_trunc(std::string const& val,unsigned width){
            return fmt::format("(uint{}_t)({})", width, val);
        }

        inline std::string create_trunc(std::string const& val,unsigned width){
            return fmt::format("(int{}_t)({})", width, val);
        }

        inline std::string create_read_mem(mem_type_e type, uint64_t addr, uint32_t size) {
            switch(size){
            case 1:
            case 2:
            case 4:
            case 8:
                return fmt::format("read_mem{}(core_ptr, {}, {}, {})", size, iss::address_type::VIRTUAL, type, addr);
            default:
                assert(false && "Unsupported mem read length");
                return "";
            }
        }

        inline std::string create_read_mem(mem_type_e type, std::string const& addr, uint32_t size) {
            switch(size){
            case 1:
            case 2:
            case 4:
            case 8:
                return fmt::format("read_mem{}(core_ptr, {}, {}, {})", size, iss::address_type::VIRTUAL, type, addr);
            default:
                assert(false && "Unsupported mem read length");
                return "";
            }
        }

        template<typename T>
        inline std::string  create_write_mem(mem_type_e type, uint64_t addr, T val) {
            switch(sizeof(T)){
            case 1:
            case 2:
            case 4:
            case 8:
                return fmt::format("  write_mem{}(core_ptr, {}, {}, {}, {});", sizeof(T), iss::address_type::VIRTUAL, type, addr, val);
            default:
                assert(false && "Unsupported mem read length");
                return "";
            }
        }

        inline std::string  create_write_mem(mem_type_e type, std::string const& addr, std::string val, unsigned size) {
            switch(size){
            case 1:
            case 2:
            case 4:
            case 8:
                return fmt::format("  write_mem{}(core_ptr, {}, {}, {}, {});", size, iss::address_type::VIRTUAL, type, addr, val);
            default:
                assert(false && "Unsupported mem read length");
                return "";
            }
        }
    };

    std::tuple<continuation_e, std::string, std::string> disass(virt_addr_t &pc) {
        unsigned cur_blk = 0;
        virt_addr_t cur_pc = pc;
        std::pair<virt_addr_t, phys_addr_t> cur_pc_mark(pc, this->core.v2p(pc));
        unsigned int num_inst = 0;
        translation_unit tu;
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
    gen_single_inst_behavior(virt_addr_t &pc_v, unsigned int &inst_cnt, translation_unit& tu) = 0;

    virtual void gen_trap_behavior(translation_unit& tu) = 0;

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

    inline void gen_sync(translation_unit& tu, sync_type s, unsigned inst_id) {
        if (s == PRE_SYNC) {
            // update icount
            tu<<"  (*icount)++;";
            tu<<"  *pc=*next_pc;";
            tu<<"  *trap_state=*pending_trap;";
            if (debugging_enabled())
                tu<<"  pre_instr_sync(vm_ptr);";
        }
        if ((s & sync_exec))
            tu("  notify_phase(core_ptr, {});", notifier_mapping[s]);
        iss::instr_info_t iinfo{cluster_id, core_id, inst_id, s};
        for (plugin_entry e : plugins) {
            if (e.sync & s)
                tu("  call_plugin((void*){}, (uint64_t){})", e.plugin_ptr, iinfo.st.value);
        }
    }

    void open_block_func(translation_unit& tu, phys_addr_t pc) {
        tu.fname = fmt::format("tcc_jit_{:#x}", pc.val);
    }

    void close_block_func(translation_unit& tu){
        tu<<"  return *next_pc;";
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
