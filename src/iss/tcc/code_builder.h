#include <iss/arch/traits.h>
#include <iss/arch_if.h>
#include <fmt/format.h>
#include <string>
#include <vector>
#include <array>

namespace iss {
namespace tcc {
enum class ICmpInst {ICMP_UGT, ICMP_ULT, ICMP_UGE, ICMP_ULE, ICMP_EQ, ICMP_NE, ICMP_SGT, ICMP_SLT, ICMP_SGE, ICMP_SLE};

std::ostream& write_prologue(std::ostream& );

struct value {
    bool is_signed() const { return type&0x100;}
    void set_signed(bool v){if(v) type|=0x100; else type&=0xff;}
    unsigned size() const { return type&0xff;}
    void set_size(unsigned s){ type = (type&0xff00)+(s&0xff);}
    value(std::string const& s, unsigned size, bool sig = false):str(s), type((sig?0x100:0)+(size&0xff)){}
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

template <typename ARCH>
struct code_builder {
    using mem_type_e  = typename arch::traits<ARCH>::mem_type_e;

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
        write_prologue(os);
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
//        if(defined_regs[arch::traits<ARCH>::LAST_BRANCH]){
//            os<<fmt::format("  uint{0}_t* reg{2:02d} = (uint{0}_t*)(regs_ptr+{1:#x});\n",
//                            arch::traits<ARCH>::reg_bit_widths[arch::traits<ARCH>::LAST_BRANCH],
//                            arch::traits<ARCH>::reg_byte_offsets[arch::traits<ARCH>::LAST_BRANCH], arch::traits<ARCH>::LAST_BRANCH);
//        }
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
        return value(fmt::format("{}", val), size/*of(T)*4*/, true);
    }

    template <typename T, typename std::enable_if<std::is_unsigned<T>::value>::type* = nullptr>
    inline value constant(T val, unsigned size) const {
        return value(fmt::format("{}U", val), size/*of(T)*4*/, false);
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
                    return value(fmt::format("(uint{}_t)((uint{}_t)({}))", size, val.size(), val), size, true);
                } else {
                    return value(fmt::format("(uint{}_t)({})", size, val), size, false);
                }
            else
                if(val.size() != size){
                    return value(fmt::format("(uint{}_t)({})", size, val), size, false);
                } else {
                    return val;
                }
        } else {
            if(!val.is_signed())
                if( val.size() != size){
                    return value(fmt::format("(int{}_t)((int{}_t)({}))", size, val.size(), val), size, true);
                } else {
                    return value(fmt::format("(int{}_t)({})", size, val), size, true);
                }
            else
                if(val.size() != size){
                    return value(fmt::format("(int{}_t)({})", size, val), size, false);
                } else {
                    return val;
                }
        }
    }

    inline value assignment(std::string const& name, value const& val, unsigned width){
        if(width==0) width=val.size();
        if(width==1){
            lines.push_back(fmt::format("bool {} = {};", name, val));
            return value(name, 1, false);
        } else{
            lines.push_back(fmt::format(val.is_signed()?"int{}_t {} = {};":"uint{}_t {} = {};",  width, name, val));
            return value(name, width, val.is_signed());
        }
    }

    inline value assignment(value const& val, unsigned width){
        return assignment(fmt::format("tmp{}", lines.size()), val, width);
    }

    inline value store(value const& val, unsigned reg_num){
        switch(reg_num){
        case arch::traits<ARCH>::NEXT_PC:
            if(val.is_signed())
                lines.push_back(fmt::format("*next_pc = (uint{}_t)({});", arch::traits<ARCH>::reg_bit_widths[reg_num], val));
            else
                lines.push_back(fmt::format("*next_pc = {};", val));
            return value("*next_pc", arch::traits<ARCH>::reg_bit_widths[reg_num], false);
        case arch::traits<ARCH>::PC:
            if(val.is_signed())
                lines.push_back(fmt::format("*pc = (uint{}_t)({});", arch::traits<ARCH>::reg_bit_widths[reg_num], val));
            else
                lines.push_back(fmt::format("*pc = {};", val));
            return value("*pc", arch::traits<ARCH>::reg_bit_widths[reg_num], false);
        default:
            defined_regs[reg_num]=true;
            if(val.is_signed())
                lines.push_back(fmt::format("*reg{:02d}  = (uint{}_t)({});", reg_num, arch::traits<ARCH>::reg_bit_widths[reg_num], val));
            else
                lines.push_back(fmt::format("*reg{:02d} = {};", reg_num, val));
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
            return value(fmt::format("(uint{}_t)({})", v.size(), v), v.size(),false);
        else
            return v;
    }

    inline value add(value const & left, value const & right){
        if(left ^ right){
            return value(fmt::format("({}) + ({})", to_uint(left), to_uint(right)), std::max(left.size(), right.size()),
                    left.is_signed() && right.is_signed());
        } else {
            return value(fmt::format("({}) + ({})", left, right), std::max(left.size(), right.size()),
                    left.is_signed() && right.is_signed());
        }
    }

    inline value sub(value const & left, value const & right){
        return value(fmt::format("({}) - ({})", left, right), std::max(left.size(), right.size()),
                left.is_signed() && right.is_signed());
    }

    inline value mul(value const & left, value const & right){
        return value(fmt::format("({}) * ({})", left, right), std::max(left.size(), right.size()),
                left.is_signed() && right.is_signed());
    }

    inline value sdiv(value const & left, value const & right){
        return value(fmt::format("({}) / ({})", left, right), std::max(left.size(), right.size()),
                left.is_signed() && right.is_signed());
    }

    inline value udiv(value const & left, value const & right){
        return value(fmt::format("({}) / ({})", left, right), std::max(left.size(), right.size()),
                left.is_signed() && right.is_signed());
    }

    inline value srem(value const & left, value const & right){
        return value(fmt::format("({}) % ({})", left, right), std::max(left.size(), right.size()),
                left.is_signed() && right.is_signed());
    }

    inline value urem(value const & left, value const & right){
        return value(fmt::format("({}) % ({})", left, right), std::max(left.size(), right.size()),
                left.is_signed() && right.is_signed());
    }

    inline value l_and(value const & left, value const & right){
        return value(fmt::format("({}) & ({})", left, right), std::max(left.size(), right.size()),
                left.is_signed() && right.is_signed());
    }

    inline value l_or(value const & left, value const & right){
        return value(fmt::format("({}) | ({})", left, right), std::max(left.size(), right.size()),
                left.is_signed() && right.is_signed());
    }

    inline value l_xor(value const & left, value const & right){
        return value(fmt::format("({}) ^ ({})", left, right), std::max(left.size(), right.size()),
                left.is_signed() && right.is_signed());
    }

    inline value b_and(value const & left, value const & right){
        return value(fmt::format("({}) && ({})", left, right), 1, false);
    }

    inline value b_or(value const & left, value const & right){
        return value(fmt::format("({}) ||({})", left, right), 1, false);
    }

    inline value l_not(value const & left){
        return value(fmt::format("~({})", left), left.size(), left.is_signed());
    }

    inline value l_neg(value const & left){
        return value(fmt::format("!({})", left), 1, false);
    }

    inline value neg(value const & left){
        return value(fmt::format("-({})", left), left.size(), false);
    }

    inline value shl(value const & val,value const & shift){
        return value(fmt::format("({})<<({})", val, shift), val.size(), val.is_signed());
    }

    inline value lshr(value const & val,value const & shift){
        if(val.is_signed())
            return value(fmt::format("((uint{}_t)({}))>>({})", val.size(), val, shift), val.size(), false);
        else
            return value(fmt::format("({})>>({})", val, shift), val.size(), false);
    }

    inline value ashr(value const & val,value const & shift){
        if(val.is_signed())
            return value(fmt::format("({})>>({})", val, shift), val.size(), false);
        else
            return value(fmt::format("((int{}_t)({}))>>({})", val.size(), val, shift), val.size(), true);
    }

    inline value choose(value const & cond, value const & left, value const & right){
        assert(left.size()==right.size());
        if(left ^right)
            return value(fmt::format("({})?({}) : ({})", cond, left, right),left.size(),left.is_signed());
        else
            return value(fmt::format("({})?({}) : ({})", cond, left, right),left.size(),left.is_signed());
    }

    inline value icmp(ICmpInst inst, value const& left, value const& right){
        switch(inst){
        case ICmpInst::ICMP_SGT:
        case ICmpInst::ICMP_UGT:
            return value(fmt::format("{} > {}", left, right), 1, false);
        case ICmpInst::ICMP_SLT:
        case ICmpInst::ICMP_ULT:
            return value(fmt::format("{} < {}", left, right), 1, false);
        case ICmpInst::ICMP_SGE:
        case ICmpInst::ICMP_UGE:
            return value(fmt::format("{} >= {}", left, right), 1, false);
        case ICmpInst::ICMP_SLE:
        case ICmpInst::ICMP_ULE:
            return value(fmt::format("{} <= {}", left, right), 1, false);
        case ICmpInst::ICMP_EQ: return value(fmt::format("{} == {}", left, right), 1, false);
        case ICmpInst::ICMP_NE: return value(fmt::format("{} != {}", left, right), 1, false);
        }
        return value("", 1, false);
    }

    inline value zext_or_trunc(value const& val, unsigned width){
        if(val.is_signed())
            return value(fmt::format("(uint{}_t)((uint{}_t)({}))", width, val.size(), val), width, false);
        else
            return value(fmt::format("(uint{}_t)({})", width, val), width, false);
    }

    inline value trunc(value const& val, unsigned width){
        if(val.is_signed())
            return value(fmt::format("(int{}_t)({})", width, val), width, true);
        else
            return value(fmt::format("(uint{}_t)({})", width, val), width, false);

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
        case 64:{
            auto id=lines.size();
            lines.push_back(fmt::format("uint{}_t rd_{};", size, id));
            if(addr.is_signed()){
                lines.push_back(fmt::format("if(read_mem{}(core_ptr, {}, {}, {}, &rd_{})) goto trap_entry;",
                        size/8, iss::address_type::VIRTUAL, type, zext_or_trunc(addr, 64), id));
                return value(fmt::format("rd_{}", id), size, false);
            } else
                lines.push_back(fmt::format("if(read_mem{}(core_ptr, {}, {}, {}, &rd_{})) goto trap_entry;",
                        size/8, iss::address_type::VIRTUAL, type, addr, id));
                return value(fmt::format("rd_{}", id), size, false);
        }
       default:
            assert(false && "Unsupported mem read length");
            return value("", 0, false);
        }
    }

    inline void  write_mem(mem_type_e type, uint64_t addr, value val) {
    	auto size = val.size();
        switch(sizeof(size)){
        case 8:
        case 16:
        case 32:
        case 64:
            lines.push_back(fmt::format("if(write_mem{}(core_ptr, {}, {}, {}, {})) goto trap_entry;", val.size()/8, iss::address_type::VIRTUAL, type, addr, val));
            break;
        default:
            assert(false && "Unsupported mem write length");
        }
    }

    inline void  write_mem(mem_type_e type, value const& addr, value val) {
        switch(val.size()){
        case 8:
        case 16:
        case 32:
        case 64:
            if(addr.is_signed())
                lines.push_back(fmt::format("if(write_mem{}(core_ptr, {}, {}, {}, {})) goto trap_entry;", val.size()/8, iss::address_type::VIRTUAL, type, zext_or_trunc(addr, 64), val));
            else
                lines.push_back(fmt::format("if(write_mem{}(core_ptr, {}, {}, {}, {})) goto trap_entry;", val.size()/8, iss::address_type::VIRTUAL, type, addr, val));
            break;
        default:
            assert(false && "Unsupported mem read length");
        }
    }

    inline value callf(std::string const& fn){
        return {fmt::format("{}()", fn), 32, false};
    }

    inline value callf(std::string const& fn, value v1){
        return {fmt::format("{}({})", fn, v1), v1.size(), v1.is_signed()};
    }

    template <typename Arg, typename... Args>
    inline value callf(std::string const& fn, Arg const& v1, Args const&... vals){
        return {fmt::format("{}({})", fn, collect_args(v1, vals...)), v1.size(), v1.is_signed()};
    }

    inline std::string collect_args(value const& v) {
        return v.str;
    }

    template <typename Arg, typename... Args>
    inline std::string collect_args(Arg const& arg, Args const&... args) {
        return fmt::format("{}, {}", arg, collect_args(args...));
    }
};


}
}

template<>
struct fmt::formatter<iss::tcc::value>
{
    template<typename ParseContext>
    constexpr typename ParseContext::iterator parse(ParseContext& ctx){
        return ctx.begin();
    }

    template<typename FormatContext>
    auto format(iss::tcc::value const& value, FormatContext& ctx) -> decltype(ctx.out()){
        return fmt::format_to(ctx.out(), "{}", value.str);
    }
};
