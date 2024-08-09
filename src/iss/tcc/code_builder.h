#include <array>
#include <cassert>
#include <cstdint>
#include <fmt/format.h>
#include <iss/arch/traits.h>
#include <iss/arch_if.h>
#include <iss/vm_jit_funcs.h>
#include <string>
#include <unordered_set>
#include <vector>

namespace iss {
namespace tcc {
enum class ICmpInst { ICMP_UGT, ICMP_ULT, ICMP_UGE, ICMP_ULE, ICMP_EQ, ICMP_NE, ICMP_SGT, ICMP_SLT, ICMP_SGE, ICMP_SLE };

struct value {
    bool is_signed() const { return type & 0x100; }
    void set_signed(bool v) {
        if(v)
            type |= 0x100;
        else
            type &= 0xff;
    }
    unsigned size() const { return type & 0xff; }
    void set_size(unsigned s) { type = (type & 0xff00) + (s & 0xff); }
    value(std::string const& s, unsigned size, bool sig = false)
    : str(s)
    , type((sig ? 0x100 : 0) + (size & 0xff)) {}
    template <typename T, typename std::enable_if<std::is_signed<T>::value>::type* = nullptr>
    value(T val)
    : value(fmt::format("{}", val), sizeof(T) * 8, true) {}
    template <typename T, typename std::enable_if<std::is_unsigned<T>::value>::type* = nullptr>
    value(T val)
    : value(fmt::format("{}", val), sizeof(T) * 8, false) {}
    value() = delete;
    value(value const&) = default;
    value(value&&) = default;
    value& operator=(value const&) = default;
    value& operator=(value&&) = default;

    bool operator^(value const& o) const { return !((is_signed() && o.is_signed()) || (!is_signed() && !o.is_signed())); }
    std::string str{};
    unsigned type{0};
};

template <typename ARCH> struct code_builder {
    using mem_type_e = typename arch::traits<ARCH>::mem_type_e;

    template <typename S, typename... Args> inline void operator()(const S& format_str, Args&&... args) {
        lines.push_back(fmt::format(format_str, args...));
    }
    inline void operator()(std::string const& s) { lines.push_back(s); }
    inline void operator()(std::string&& s) { lines.push_back(s); }
    std::string fname;
    std::vector<std::string> lines{};
    std::unordered_set<std::string> additional_prologue;
    std::array<bool, arch::traits<ARCH>::NUM_REGS> defined_regs{false};
    inline std::string add_reg_ptr(std::string const& name, unsigned reg_num) {
        return fmt::format("  uint{0}_t* {2} = (uint{0}_t*)(regs_ptr+{1:#x});\n", arch::traits<ARCH>::reg_bit_widths[reg_num],
                           arch::traits<ARCH>::reg_byte_offsets[reg_num], name);
    }
    std::ostream& write_prologue(std::ostream&);
    std::string finish() {
        std::ostringstream os;
        // generate prologue
        write_prologue(os);
        os << fmt::format("uint64_t {}(uint8_t* regs_ptr, void* core_ptr, void* vm_ptr) __attribute__ ((regnum(3)))  {{\n", fname);
        os << add_reg_ptr("pc", arch::traits<ARCH>::PC);
        os << add_reg_ptr("next_pc", arch::traits<ARCH>::NEXT_PC);
        os << add_reg_ptr("trap_state", arch::traits<ARCH>::TRAP_STATE);
        os << add_reg_ptr("pending_trap", arch::traits<ARCH>::PENDING_TRAP);
        os << add_reg_ptr("icount", arch::traits<ARCH>::ICOUNT);
        os << add_reg_ptr("last_branch", arch::traits<ARCH>::LAST_BRANCH);
        os << "*last_branch = 0;\n";
        os << "uint64_t tval = 0;\n";

        for(size_t i = 0; i < arch::traits<ARCH>::NUM_REGS; ++i) {
            if(defined_regs[i]) {
                os << fmt::format("  uint{0}_t* reg{2:02d} = (uint{0}_t*)(regs_ptr+{1:#x});\n", arch::traits<ARCH>::reg_bit_widths[i],
                                  arch::traits<ARCH>::reg_byte_offsets[i], i);
            }
        }
        // add generated code
        std::copy(lines.begin(), lines.end(), std::ostream_iterator<std::string>(os, "\n"));
        // and the epilogue
        os << "}";
        return os.str();
    }

    void open_scope() { lines.push_back("{"); }

    void close_scope() { lines.push_back("}"); }
    void add_prologue(std::string const& str) { additional_prologue.insert(str); }
    inline void open_if(value const& cond) { lines.push_back(fmt::format("if({}){{", cond)); }
    inline void open_else() { lines.push_back("}else{"); }

    template <typename T, typename std::enable_if<std::is_signed<T>::value>::type* = nullptr>
    inline value constant(T val, unsigned size) const {
        return value(fmt::format("{}", val), size /*of(T)*4*/, true);
    }

    template <typename T, typename std::enable_if<std::is_unsigned<T>::value>::type* = nullptr>
    inline value constant(T val, unsigned size) const {
        return value(fmt::format("{}U", val), size /*of(T)*4*/, false);
    }

    template <typename T, typename std::enable_if<std::is_integral<T>::value>::type* = nullptr>
    inline value ext(T val, unsigned size, bool isSigned) const {
        if(!isSigned)
            return value(fmt::format("(uint{}_t)({})", size, val), size, false);
        else
            return value(fmt::format("(int{}_t)({})", size, val), size, true);
    }

    inline value ext(value const& val, unsigned target_size, bool target_is_signed) const {
        if(val.is_signed()) {
            if(target_size == val.size())
                if(target_is_signed)
                    return val; // intx_t -> intx_t
                else
                    return value(fmt::format("(uint{}_t)({})", target_size, val), target_size,
                                 false); // intx_t -> uintx_t
            else if(target_is_signed)
                return value(fmt::format("(int{}_t)({})", target_size, val), target_size, true); // intx_t -> inty_t
            else
                return value(fmt::format("(uint{}_t)((int{}_t)({}))", target_size, target_size, val), target_size, false);
            ; // intx_t -> inty_t -> uinty_t
        } else {
            if(target_size == val.size())
                if(target_is_signed)
                    return value(fmt::format("(int{}_t)({})", target_size, val), target_size, true); // uintx_t ->
                                                                                                     // intx_t
                else
                    return val; // uintx_t -> uintx_t
            else if(target_is_signed)
                return value(fmt::format("(int{}_t)((uint{}_t)({}))", target_size, target_size, val), target_size,
                             true); // uintx_t -> uinty_t -> inty_t
            else
                return value(fmt::format("(uint{}_t)({})", target_size, val), target_size, false); // uintx_t -> uinty_t
        }
    }

    inline value assignment(std::string const& name, value const& val, unsigned width) {
        if(width == 0)
            width = val.size();
        if(width == 1) {
            lines.push_back(fmt::format("bool {} = {};", name, val));
            return value(name, 1, false);
        } else {
            lines.push_back(fmt::format(val.is_signed() ? "int{}_t {} = {};" : "uint{}_t {} = {};", width, name, val));
            return value(name, width, val.is_signed());
        }
    }

    inline value assignment(value const& val, unsigned width) { return assignment(fmt::format("tmp{}", lines.size()), val, width); }

    inline value store(unsigned reg_num, value const& val) {
        switch(reg_num) {
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
        case arch::traits<ARCH>::LAST_BRANCH:
            lines.push_back(fmt::format("*last_branch = {};", val));
            return value("*last_branch", arch::traits<ARCH>::reg_bit_widths[reg_num], false);
        default:
            defined_regs[reg_num] = true;
            if(val.is_signed())
                lines.push_back(fmt::format("*reg{:02d}  = (uint{}_t)({});", reg_num, arch::traits<ARCH>::reg_bit_widths[reg_num], val));
            else
                lines.push_back(fmt::format("*reg{:02d} = {};", reg_num, val));
            return value(fmt::format("*reg{:02d}", reg_num), arch::traits<ARCH>::reg_bit_widths[reg_num], false);
        }
    }

    inline value load(unsigned reg_num, unsigned nesting_lvl) {
        switch(reg_num) {
        case arch::traits<ARCH>::NEXT_PC:
            return value("(*next_pc)", arch::traits<ARCH>::reg_bit_widths[reg_num], false);
        case arch::traits<ARCH>::PC:
            return value("(*pc)", arch::traits<ARCH>::reg_bit_widths[reg_num], false);
        default:
            defined_regs[reg_num] = true;
            return value(fmt::format("(*reg{:02d})", reg_num), arch::traits<ARCH>::reg_bit_widths[reg_num], false);
        }
    }

    inline value to_int(value const& v) {
        if(v.is_signed())
            return value(fmt::format("(int{}_t)({})", v.size(), v), v.size(), false);
        else
            return v;
    }

    inline value add(value const& left, value const& right) {
        if(left ^ right) {
            return value(fmt::format("({}) + ({})", to_int(left), to_int(right)), std::max(left.size(), right.size()),
                         left.is_signed() && right.is_signed());
        } else {
            return value(fmt::format("({}) + ({})", left, right), std::max(left.size(), right.size()),
                         left.is_signed() && right.is_signed());
        }
    }

    inline value sub(value const& left, value const& right) {
        return value(fmt::format("({}) - ({})", left, right), std::max(left.size(), right.size()), left.is_signed() && right.is_signed());
    }

    inline value mul(value const& left, value const& right) {
        auto target_size = std::max(left.size(), right.size()) * 2;
        auto target_type_left = left.is_signed() ? fmt::format("(int{}_t)", target_size) : fmt::format("(uint{}_t)", target_size);
        auto target_type_right = right.is_signed() ? fmt::format("(int{}_t)", target_size) : fmt::format("(uint{}_t)", target_size);
        return value(fmt::format("({}{}) * ({}{})", target_type_left, left, target_type_right, right), target_size,
                     left.is_signed() && right.is_signed());
    }

    inline value sdiv(value const& left, value const& right) {
        return value(fmt::format("({}) / ({})", left, right), std::max(left.size(), right.size()), left.is_signed() && right.is_signed());
    }

    inline value udiv(value const& left, value const& right) {
        return value(fmt::format("({}) / ({})", left, right), std::max(left.size(), right.size()), left.is_signed() && right.is_signed());
    }

    inline value srem(value const& left, value const& right) {
        return value(fmt::format("(({}) % ({}))", left, right), std::max(left.size(), right.size()), left.is_signed() && right.is_signed());
    }

    inline value urem(value const& left, value const& right) {
        return value(fmt::format("(({}) % ({}))", left, right), std::max(left.size(), right.size()), left.is_signed() && right.is_signed());
    }

    inline value bitwise_and(value const& left, value const& right) {
        return value(fmt::format("(({}) & ({}))", left, right), std::max(left.size(), right.size()), left.is_signed() && right.is_signed());
    }

    inline value bitwise_or(value const& left, value const& right) {
        return value(fmt::format("(({}) | ({}))", left, right), std::max(left.size(), right.size()), left.is_signed() && right.is_signed());
    }

    inline value bitwise_xor(value const& left, value const& right) {
        return value(fmt::format("(({}) ^ ({}))", left, right), std::max(left.size(), right.size()), left.is_signed() && right.is_signed());
    }

    inline value logical_and(value const& left, value const& right) { return value(fmt::format("({}) && ({})", left, right), 1, false); }

    inline value logical_or(value const& left, value const& right) { return value(fmt::format("({}) ||({})", left, right), 1, false); }

    inline value logical_not(value const& left) { return value(fmt::format("!({})", left), 1, false); }

    inline value logical_neg(value const& left) { return value(fmt::format("~({})", left), left.size(), left.is_signed()); }

    inline value neg(value const& left) { return value(fmt::format("-({})", left), left.size(), false); }
    inline value postIncrement(value const& val) { return value(fmt::format("({})++", val), val.size(), val.is_signed()); }
    inline value postDecrement(value const& val) { return value(fmt::format("({})--", val), val.size(), val.is_signed()); }
    inline value preIncrement(value const& val) { return value(fmt::format("++({})", val), val.size(), val.is_signed()); }
    inline value preDecrement(value const& val) { return value(fmt::format("--({})", val), val.size(), val.is_signed()); }
    inline value shl(value const& val, value const& shift) {
        return value(fmt::format("({})<<({})", val, shift), val.size(), val.is_signed());
    }

    inline value lshr(value const& val, value const& shift) {
        if(val.is_signed())
            return value(fmt::format("((uint{}_t)({}))>>({})", val.size(), val, shift), val.size(), false);
        else
            return value(fmt::format("({})>>({})", val, shift), val.size(), false);
    }

    inline value ashr(value const& val, value const& shift) {
        if(val.is_signed())
            return value(fmt::format("({})>>({})", val, shift), val.size(), false);
        else
            return value(fmt::format("((int{}_t)({}))>>({})", val.size(), val, shift), val.size(), true);
    }

    inline value conditionalAssignment(value const& cond, value const& left, value const& right) {
        assert(left.size() == right.size());
        if(left ^ right)
            return value(fmt::format("({})?({}) : ({})", cond, left, right), left.size(), left.is_signed());
        else
            return value(fmt::format("({})?({}) : ({})", cond, left, right), left.size(), left.is_signed());
    }

    inline value icmp(ICmpInst inst, value const& left, value const& right) {
        switch(inst) {
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
        case ICmpInst::ICMP_EQ:
            return value(fmt::format("{} == {}", left, right), 1, false);
        case ICmpInst::ICMP_NE:
            return value(fmt::format("{} != {}", left, right), 1, false);
        }
        return value("", 1, false);
    }

    inline value slice(value val, uint32_t bit, uint32_t width) {
        assert(((bit + width) <= val.size()) && "Invalid slice range");
        // analog to bit_sub in scc util
        // T res = (v >> bit) & ((T(1) << width) - 1);
        value res = bitwise_and(lshr(val, bit), ((1 << width) - 1));
        return res;
    }

    inline value read_mem(mem_type_e type, uint64_t addr, uint32_t size) {
        switch(size) {
        case 8:
        case 16:
        case 32:
        case 64: {
            auto id = lines.size();
            lines.push_back(fmt::format("uint{}_t rd_{};", size, id));
            lines.push_back(fmt::format("if((*read_mem{})(core_ptr, {}, {}, {}, &rd_{})) goto trap_entry;", size / 8,
                                        iss::address_type::VIRTUAL, type, addr, id));
            return value(fmt::format("rd_{}", id), size, false);
        }
        default:
            assert(false && "Unsupported mem read length");
            return value("", 0, false);
        }
    }

    inline value read_mem(mem_type_e type, value const& addr, uint32_t size) {
        switch(size) {
        case 8:
        case 16:
        case 32:
        case 64: {
            auto id = lines.size();
            lines.push_back(fmt::format("uint{}_t rd_{};", size, id));
            lines.push_back(fmt::format("if((*read_mem{})(core_ptr, {}, {}, {}, &rd_{})) goto trap_entry;", size / 8,
                                        iss::address_type::VIRTUAL, type, addr, id));
            return value(fmt::format("rd_{}", id), size, false);
        }
        default:
            assert(false && "Unsupported mem read length");
            return value("", 0, false);
        }
    }

    inline void write_mem(mem_type_e type, uint64_t addr, value val) {
        switch(val.size()) {
        case 8:
        case 16:
        case 32:
        case 64:
            lines.push_back(fmt::format("if((*write_mem{})(core_ptr, {}, {}, {}, {})) goto trap_entry;", val.size() / 8,
                                        iss::address_type::VIRTUAL, type, addr, val));
            break;
        default:
            assert(false && "Unsupported mem write length");
        }
    }

    inline void write_mem(mem_type_e type, value const& addr, value val) {
        switch(val.size()) {
        case 8:
        case 16:
        case 32:
        case 64:
            lines.push_back(fmt::format("if((*write_mem{})(core_ptr, {}, {}, {}, {})) goto trap_entry;", val.size() / 8,
                                        iss::address_type::VIRTUAL, type, addr, val));
            break;
        default:
            assert(false && "Unsupported mem read length");
        }
    }

    inline value callf(std::string const& fn) { return {fmt::format("(*{})()", fn), 32, false}; }

    inline value callf(std::string const& fn, value v1) { return {fmt::format("(*{})({})", fn, v1), v1.size(), v1.is_signed()}; }

    template <typename Arg, typename... Args> inline value callf(std::string const& fn, Arg const& v1, Args const&... vals) {
        return {fmt::format("(*{})({})", fn, collect_args(v1, vals...)), v1.size(), v1.is_signed()};
    }

    inline std::string collect_args(value const& v) { return v.str; }

    template <typename Arg, typename... Args> inline std::string collect_args(Arg const& arg, Args const&... args) {
        return fmt::format("{}, {}", arg, collect_args(args...));
    }
};

template <typename ARCH> inline std::ostream& code_builder<ARCH>::write_prologue(std::ostream& os) {
    os << "#define bool    _Bool\n";
    os << "#define true    1\n";
    os << "#define false   0\n";
    os << "typedef __SIZE_TYPE__ size_t;\n";
    os << "typedef __PTRDIFF_TYPE__ ssize_t;\n";
    os << "typedef __WCHAR_TYPE__ wchar_t;\n";
    os << "typedef __PTRDIFF_TYPE__ ptrdiff_t;\n";
    os << "typedef __PTRDIFF_TYPE__ intptr_t;\n";
    os << "typedef __SIZE_TYPE__ uintptr_t;\n";
    os << "#define __int8_t_defined\n";
    os << "typedef signed char int8_t;\n";
    os << "typedef signed short int int16_t;\n";
    os << "typedef signed int int32_t;\n";
    os << "#ifdef __LP64__\n";
    os << "typedef signed long int int64_t;\n";
    os << "#else\n";
    os << "typedef signed long long int int64_t;\n";
    os << "#endif\n";
    os << "typedef unsigned char uint8_t;\n";
    os << "typedef unsigned short int uint16_t;\n";
    os << "typedef unsigned int uint32_t;\n";
    os << "#ifdef __LP64__\n";
    os << "typedef unsigned long int uint64_t;\n";
    os << "#else\n";
    os << "typedef unsigned long long int uint64_t;\n";
    os << "#endif\n";
    os << "#define NULL ((void*)0)\n";
    os << "#define offsetof(type, field) ((size_t)&((type *)0)->field)\n";
    os << "#if defined (__need_wint_t)\n";
    os << "#ifndef _WINT_T\n";
    os << "#define _WINT_T\n";
    os << "typedef __WINT_TYPE__ wint_t;\n";
    os << "#endif\n";
    os << "#undef __need_wint_t\n";
    os << "#endif\n";

    os << "int (*read_mem1)( void*, uint32_t, uint32_t, uint64_t, uint8_t*)=" << (uintptr_t)&read_mem1 << ";\n";
    os << "int (*write_mem1)(void*, uint32_t, uint32_t, uint64_t, uint8_t)=" << (uintptr_t)&write_mem1 << ";\n";
    os << "int (*read_mem2)( void*, uint32_t, uint32_t, uint64_t, uint16_t*)=" << (uintptr_t)&read_mem2 << ";\n";
    os << "int (*write_mem2)(void*, uint32_t, uint32_t, uint64_t, uint16_t)=" << (uintptr_t)&write_mem2 << ";\n";
    os << "int (*read_mem4)( void*, uint32_t, uint32_t, uint64_t, uint32_t*)=" << (uintptr_t)&read_mem4 << ";\n";
    os << "int (*write_mem4)(void*, uint32_t, uint32_t, uint64_t, uint32_t)=" << (uintptr_t)&write_mem4 << ";\n";
    os << "int (*read_mem8)( void*, uint32_t, uint32_t, uint64_t, uint64_t*)=" << (uintptr_t)&read_mem8 << ";\n";
    os << "int (*write_mem8)(void*, uint32_t, uint32_t, uint64_t, uint64_t)=" << (uintptr_t)&write_mem8 << ";\n";
    os << "uint64_t (*enter_trap)(void*, uint64_t, uint64_t, uint64_t)=" << (uintptr_t)&enter_trap << ";\n";
    os << "uint64_t (*leave_trap)(void*, uint64_t)=" << (uintptr_t)&leave_trap << ";\n";
    os << "void (*wait)(void*, uint64_t)=" << (uintptr_t)&wait << ";\n";
    os << "void (*print_string)(void*, char*)=" << (uintptr_t)&print_string << ";\n";
    os << "void (*print_disass)(void*, uint64_t, char*)=" << (uintptr_t)&print_disass << ";\n";
    os << "void (*pre_instr_sync)(void*)=" << (uintptr_t)&pre_instr_sync << ";\n";
    os << "void (*notify_phase)(void*, uint32_t)=" << (uintptr_t)&notify_phase << ";\n";
    os << "void (*call_plugin)(void*, uint64_t)=" << (uintptr_t)&call_plugin << ";\n";
    for(auto& line : additional_prologue) {
        os << line << "\n";
    }
    return os;
}
} // namespace tcc
} // namespace iss

template <> struct fmt::formatter<iss::tcc::value> {
    template <typename ParseContext> constexpr typename ParseContext::iterator parse(ParseContext& ctx) { return ctx.begin(); }

    template <typename FormatContext> auto format(iss::tcc::value const& value, FormatContext& ctx) -> decltype(ctx.out()) {
        return fmt::format_to(ctx.out(), "{}", value.str);
    }
};
