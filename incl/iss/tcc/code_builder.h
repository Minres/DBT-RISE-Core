#include <iss/arch/traits.h>
#include <iss/arch_if.h>
#include <fmt/format.h>
#include <string>
#include <vector>
#include <array>

namespace iss {
namespace tcc {
enum class ICmpInst {ICMP_UGT, ICMP_ULT, ICMP_UGE, ICMP_ULE, ICMP_EQ, ICMP_NE, ICMP_SGT, ICMP_SLT, ICMP_SGE, ICMP_SLE};
template <typename ARCH>
struct code_builder {
    using mem_type_e  = typename arch::traits<ARCH>::mem_type_e;

    struct value {
        bool is_signed() const { return type&0x100;}
        void set_signed(bool v){if(v) type|=0x100; else type&=0xff;}
        unsigned size() const { return type&0xff;}
        void set_size(unsigned s){ type = (type&0xff00)+(s&0xff);}
        value(std::string const& s, unsigned size, bool sig):str(s), type((sig?0x100:0)+(size&0xff)){}
        value() = delete;
        value(value const&) = default;
        value(value&&) = default;
        value& operator=(value const&) = default;
        value& operator=(value &&) = default;

        bool operator^(value const& o) const {
            return !((is_signed()&& o.is_signed()) || (!is_signed()&& !o.is_signed()));
        }
        std::string str{};
        unsigned type{0};
    };

    template <typename S, typename... Args>
    inline void operator()(const S& format_str, Args&&... args) {
        lines.push_back(fmt::format(format_str, args...));
    }
    inline void operator()(std::string const& s){ lines.push_back(s);}
    inline void operator()(std::string && s){ lines.push_back(s);}
//        inline void operator<<(std::string const& s){ lines.push_back(s);}
//        inline void operator<<(std::string && s){ lines.push_back(s);}
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
        lines.push_back("{");
    }

    void close_scope() {
        lines.push_back("}");
    }

    template <typename T, typename std::enable_if<std::is_signed<T>::value>::type* = nullptr>
    inline value constant(T val, unsigned size) const {
        return value(fmt::format("{}", val), sizeof(T)*4, true);
    }

    template <typename T, typename std::enable_if<std::is_unsigned<T>::value>::type* = nullptr>
    inline value constant(T val, unsigned size) const {
        return value(fmt::format("{}U", val), sizeof(T)*4, false);
    }


    template <typename T, typename std::enable_if<std::is_integral<T>::value>::type* = nullptr>
    inline value ext(T val, unsigned size, bool isUnSigned) const {
        if(isUnSigned)
            return value(fmt::format("(uint{}_t)({})", size, val), size, false);
        else
            return value(fmt::format("(int{}_t)({})", size, val), size, true);

    }

    inline value ext(value const& val, unsigned size, bool isUnSigned) const {
        if(isUnSigned){
            if(val.is_signed())
                if(val.size() != size){
                    return value(fmt::format("(uint{}_t)((uint{}_t)({}))", size, val.size(), val.str), size, true);
                } else {
                    return value(fmt::format("(uint{}_t)({})", size, val.str), size, false);
                }
            else
                return val;
        } else {
            if(!val.is_signed())
                if( val.size() != size){
                    return value(fmt::format("(int{}_t)((int{}_t)({}))", size, val.size(), val.str), size, true);
                } else {
                    return value(fmt::format("(int{}_t)({})", size, val.str), size, true);
                }
            else
                return val;
        }
    }

    inline value assignment(std::string const& name, value const& val, unsigned width = 0){
        if(width==0) width=val.size();
        if(width==1){
            lines.push_back(fmt::format("bool {} = {};", name, val.str));
            return value(name, 1, false);
        } else{
            lines.push_back(fmt::format(val.is_signed()?"int{}_t {} = {};":"uint{}_t {} = {};",  width, name, val.str));
            return value(name, width, val.is_signed());
        }
    }

    inline value store(value const& val, unsigned reg_num){
        switch(reg_num){
        case arch::traits<ARCH>::NEXT_PC:
            if(val.is_signed())
                lines.push_back(fmt::format("*next_pc = (uint{}_t)({});", arch::traits<ARCH>::reg_bit_widths[reg_num], val.str));
            else
                lines.push_back(fmt::format("*next_pc = {};", val.str));
            return value("*next_pc", arch::traits<ARCH>::reg_bit_widths[reg_num], false);
        case arch::traits<ARCH>::PC:
            if(val.is_signed())
                lines.push_back(fmt::format("*pc = (uint{}_t)({});", arch::traits<ARCH>::reg_bit_widths[reg_num], val.str));
            else
                lines.push_back(fmt::format("*pc = {};", val.str));
            return value("*pc", arch::traits<ARCH>::reg_bit_widths[reg_num], false);
        default:
            defined_regs[reg_num]=true;
            if(val.is_signed())
                lines.push_back(fmt::format("*reg{:02d}  = (uint{}_t)({});", reg_num, arch::traits<ARCH>::reg_bit_widths[reg_num], val.str));
            else
                lines.push_back(fmt::format("*reg{:02d} = {};", reg_num, val.str));
            return value(fmt::format("*reg{:02d}", reg_num), arch::traits<ARCH>::reg_bit_widths[reg_num], false);
        }
    }

    inline value load(unsigned reg_num, unsigned nesting_lvl){
        switch(reg_num){
        case arch::traits<ARCH>::NEXT_PC:
            return value("(*next_pc)", arch::traits<ARCH>::reg_bit_widths[reg_num], false);
        case arch::traits<ARCH>::PC:
            return value("(*pc)", arch::traits<ARCH>::reg_bit_widths[reg_num], false);
        default:
            defined_regs[reg_num]=true;
            return value(fmt::format("(*reg{:02d})", reg_num), arch::traits<ARCH>::reg_bit_widths[reg_num], false);
        }
    }

    inline value to_uint(value const & v){
        if(v.is_signed())
            return value(fmt::format("(uint{}_t)({})", v.size(), v.str), v.size(),false);
        else
            return v;
    }

    inline value add(value const & left, value const & right){
        if(left ^ right){
            return value(fmt::format("({}) + ({})", to_uint(left).str, to_uint(right).str), std::max(left.size(), right.size()),
                    left.is_signed() && right.is_signed());
        } else {
            return value(fmt::format("({}) + ({})", left.str, right.str), std::max(left.size(), right.size()),
                    left.is_signed() && right.is_signed());
        }
    }

    inline value sub(value const & left, value const & right){
        return value(fmt::format("({}) - ({})", left.str, right.str), std::max(left.size(), right.size()),
                left.is_signed() && right.is_signed());
    }

    inline value l_and(value const & left, value const & right){
        return value(fmt::format("({}) & ({})", left.str, right.str), std::max(left.size(), right.size()),
                left.is_signed() && right.is_signed());
    }

    inline value l_or(value const & left, value const & right){
        return value(fmt::format("({}) | ({})", left.str, right.str), std::max(left.size(), right.size()),
                left.is_signed() && right.is_signed());
    }

    inline value l_xor(value const & left, value const & right){
        return value(fmt::format("({}) ^ ({})", left.str, right.str), std::max(left.size(), right.size()),
                left.is_signed() && right.is_signed());
    }

    inline value b_and(value const & left, value const & right){
        return value(fmt::format("({}) && ({})", left.str, right.str), 1, false);
    }

    inline value b_or(value const & left, value const & right){
        return value(fmt::format("({}) ||({})", left.str, right.str), 1, false);
    }

    inline value l_not(value const & left){
        return value(fmt::format("~({})", left.str), left.size(), left.is_signed());
    }

    inline value l_neg(value const & left){
        return value(fmt::format("!({})", left.str), 1, false);
    }

    inline value neg(value const & left){
        return value(fmt::format("-({})", left.str), 1, false);
    }

    inline value shl(value const & val,value const & shift){
        return value(fmt::format("({})<<({})", val.str, shift.str), val.size(), val.is_signed());
    }

    inline value lshr(value const & val,value const & shift){
        if(val.is_signed())
            return value(fmt::format("((uint{}_t)({}))>>({})", val.size(), val.str, shift.str), val.size(), false);
        else
            return value(fmt::format("({})>>({})", val.str, shift.str), val.size(), false);
    }

    inline value ashr(value const & val,value const & shift){
        if(val.is_signed())
            return value(fmt::format("({})>>({})", val.str, shift.str), val.size(), false);
        else
            return value(fmt::format("((int{}_t)({}))>>({})", val.size(), val.str, shift.str), val.size(), true);
    }

    inline value choose(value const & cond, value const & left, value const & right){
        assert(left.size()==right.size());
        if(left ^right)
            return value(fmt::format("({})?({}) : ({})", cond.str, left.str, right.str),left.size(),left.is_signed());
        else
            return value(fmt::format("({})?({}) : ({})", cond.str, left.str, right.str),left.size(),left.is_signed());
    }

    inline value icmp(ICmpInst inst, value const& left, value const& right){
        switch(inst){
        case ICmpInst::ICMP_SGT:
        case ICmpInst::ICMP_UGT:
            return value(fmt::format("{} > {}", left.str, right.str), 1, false);
        case ICmpInst::ICMP_SLT:
        case ICmpInst::ICMP_ULT:
            return value(fmt::format("{} < {}", left.str, right.str), 1, false);
        case ICmpInst::ICMP_SGE:
        case ICmpInst::ICMP_UGE:
            return value(fmt::format("{} >= {}", left.str, right.str), 1, false);
        case ICmpInst::ICMP_SLE:
        case ICmpInst::ICMP_ULE:
            return value(fmt::format("{} <= {}", left.str, right.str), 1, false);
        case ICmpInst::ICMP_EQ: return value(fmt::format("{} == {}", left.str, right.str), 1, false);
        case ICmpInst::ICMP_NE: return value(fmt::format("{} != {}", left.str, right.str), 1, false);
        }
        return value("", 1, false);
    }

    inline value zext_or_trunc(value const& val, unsigned width){
        if(val.is_signed())
            return value(fmt::format("(uint{}_t)((uint{}_t)({}))", width, val.size(), val.str), width, false);
        else
            return value(fmt::format("(uint{}_t)({})", width, val.str), width, false);
    }

    inline value trunc(value const& val, unsigned width){
        if(val.is_signed())
            return value(fmt::format("(int{}_t)({})", width, val.str), width, true);
        else
            return value(fmt::format("(uint{}_t)({})", width, val.str), width, false);

    }

    inline value read_mem(mem_type_e type, uint64_t addr, uint32_t size) {
        switch(size){
        case 8:
        case 16:
        case 32:
        case 64:
            lines.push_back(fmt::format("uint{}_t rd_{} = read_mem{}(core_ptr, {}, {}, {});",
                    size, lines.size(), size/8, iss::address_type::VIRTUAL, type, addr));
            return value(fmt::format("rd_{}", lines.size()-1), size, false);
        default:
            assert(false && "Unsupported mem read length");
            return value("", 0, false);
        }
    }

    inline value read_mem(mem_type_e type, value const& addr, uint32_t size) {
        switch(size){
        case 8:
        case 16:
        case 32:
        case 64:
            if(addr.is_signed()){
                lines.push_back(fmt::format("uint{}_t rd_{} = read_mem{}(core_ptr, {}, {}, {});",
                        size, lines.size(), size/8, iss::address_type::VIRTUAL, type, zext_or_trunc(addr, 64).str));
                return value(fmt::format("rd_{}", lines.size()-1), size, false);
            } else
                lines.push_back(fmt::format("uint{}_t rd_{} = read_mem{}(core_ptr, {}, {}, {});",
                        size, lines.size(), size/8, iss::address_type::VIRTUAL, type, addr.str));
                return value(fmt::format("rd_{}", lines.size()-1), size, false);
       default:
            assert(false && "Unsupported mem read length");
            return value("", 0, false);
        }
    }

    inline void  write_mem(mem_type_e type, uint64_t addr, value val) {
        switch(sizeof(val.size)){
        case 8:
        case 16:
        case 32:
        case 64:
            lines.push_back(fmt::format("write_mem{}(core_ptr, {}, {}, {}, {});", val.size()/8, iss::address_type::VIRTUAL, type, addr, val.str));
            break;
        default:
            assert(false && "Unsupported mem read length");
        }
    }

    inline void  write_mem(mem_type_e type, value const& addr, value val) {
        switch(val.size()){
        case 8:
        case 16:
        case 32:
        case 64:
            if(addr.is_signed())
                lines.push_back(fmt::format("write_mem{}(core_ptr, {}, {}, {}, {});", val.size()/8, iss::address_type::VIRTUAL, type, zext_or_trunc(addr, 64).str, val.str));
            else
                lines.push_back(fmt::format("write_mem{}(core_ptr, {}, {}, {}, {});", val.size()/8, iss::address_type::VIRTUAL, type, addr.str, val.str));
            break;
        default:
            assert(false && "Unsupported mem read length");
        }
    }
};


}
}
