/*******************************************************************************
 * Copyright (C) 2024 MINRES Technologies GmbH
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

#ifndef ASMJIT_VM_UTIL_H_
#define ASMJIT_VM_UTIL_H_

#include <asmjit/asmjit.h>
#include <cassert>
#include <fmt/core.h>
#include <stdexcept>
#include <nonstd/variant.hpp>
#include <type_traits>
#if __cplusplus<201703L
namespace std {
template<bool _Cond, typename _Tp = void>
  using enable_if_t = typename std::enable_if<_Cond, _Tp>::type;
template <typename _Tp>
  using is_integral_v = typename is_integral<_Tp>::value;
}
#endif

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
using x86_reg_t = nonstd::variant<x86::Gp, dGp>;

// Wrapper for cc.mov to allow more flexible types
void mov(x86::Compiler& cc, x86_reg_t _dest, x86_reg_t _source);
void mov(x86::Compiler& cc, x86_reg_t dest, x86::Mem source);
void mov(x86::Compiler& cc, x86::Mem dest, x86_reg_t source);
void mov(x86::Compiler& cc, x86::Mem dest, x86::Gp source);
// Integral -> x86_reg_t
template <typename T, typename = std::enable_if_t<std::is_integral<T>::value>> void mov(x86::Compiler& cc, x86_reg_t dest, T source);
// Integral -> x86::Mem
template <typename T, typename = std::enable_if_t<std::is_integral<T>::value>> void mov(x86::Compiler& cc, x86::Mem dest, T source);

// Wrapper for cc.cmp to allow more flexible types
void cmp(x86::Compiler& cc, x86_reg_t a, x86_reg_t b);
void cmp(x86::Compiler& cc, x86_reg_t a, x86::Mem b);
void cmp(x86::Compiler& cc, x86::Mem a, x86_reg_t b);
// cmp x86_reg_t and Integral
template <typename T, typename = std::enable_if_t<std::is_integral<T>::value>> void cmp(x86::Compiler& cc, x86_reg_t _a, T b);
// cmp x86::Mem and Integral || x86::Mem || x86:.Gp
template <typename T, typename = std::enable_if_t<std::is_integral<T>::value || std::is_same<T, x86::Mem>::value || std::is_same<T, x86::Gp>::value>>
void cmp(x86::Compiler& cc, x86::Mem a, T b);

// Functions for creation of x86::Gp and dGp
x86_reg_t get_reg(x86::Compiler& cc, unsigned size, bool is_signed = true);
x86::Gp get_reg_Gp(x86::Compiler& cc, unsigned size, bool is_signed = true);
dGp get_reg_dGp(x86::Compiler& cc, unsigned size, bool is_signed = true);

// Truncation and (sign) extension
x86_reg_t gen_ext(x86::Compiler& cc, x86_reg_t _val, unsigned target_size, bool is_signed);
x86::Gp gen_ext_Gp(x86::Compiler& cc, x86_reg_t _val, unsigned size, bool is_signed);
dGp gen_ext_dGp(x86::Compiler& cc, x86_reg_t _val, unsigned size, bool is_signed);
// Extension (and conversion to x86::Gp or dGp) Integral
template <typename T, typename = std::enable_if_t<std::is_integral<T>::value>>
inline x86_reg_t gen_ext(x86::Compiler& cc, T val, unsigned target_size, bool is_signed);

// Mathematical and binary operations
enum operation { add, sub, band, bor, bxor, shl, sar, shr };
enum complex_operation { smul, umul, sumul, sdiv, udiv, srem, urem };
enum comparison_operation { land, lor, eq, ne, lt, ltu, gt, gtu, lte, lteu, gte, gteu };
enum unary_operation { lnot, inc, dec, bnot, neg };

// Helper for signed division
void expand_division_operand(x86::Compiler& cc, x86::Gp upper_half, x86::Gp dividend);
// Helper for mulitplication
x86_reg_t _multiply(x86::Compiler& cc, complex_operation op, x86_reg_t _a, x86_reg_t _b);

/*
Type operation
*/
x86_reg_t gen_operation(x86::Compiler& cc, operation op, x86_reg_t _a, x86_reg_t _b);
// x86_reg_t and Integral
template <typename T, typename = std::enable_if_t<std::is_integral<T>::value>>
x86_reg_t gen_operation(x86::Compiler& cc, operation op, x86_reg_t _a, T b);
// x86::Gp and Integral || x86::Gp
template <typename T, typename = std::enable_if_t<std::is_integral<T>::value || std::is_same<T, x86::Gp>::value>>
x86::Gp gen_operation_Gp(x86::Compiler& cc, operation op, x86::Gp a, T b);

/*
Type complex_operation
*/
x86_reg_t gen_operation(x86::Compiler& cc, complex_operation op, x86_reg_t _a, x86_reg_t _b);
// Integral and Integral
template <typename V, typename W, typename = std::enable_if_t<std::is_integral<V>::value && std::is_integral<W>::value>>
x86_reg_t gen_operation(x86::Compiler& cc, complex_operation op, V a, W b);
// x86_reg_t and Integral
template <typename T, typename = std::enable_if_t<std::is_integral<T>::value>>
x86_reg_t gen_operation(x86::Compiler& cc, complex_operation op, x86_reg_t a, T b);
// x86::Gp and Integral || x86::Gp
template <typename T, typename = std::enable_if_t<std::is_integral<T>::value || std::is_same<T, x86::Gp>::value>>
x86::Gp gen_operation_Gp(x86::Compiler& cc, comparison_operation op, x86::Gp a, T b);

/*
Type comparison_operation
*/
x86_reg_t gen_operation(x86::Compiler& cc, comparison_operation op, x86_reg_t _a, x86_reg_t _b);
x86::Gp gen_operation_Gp(x86::Compiler& cc, comparison_operation op, x86::Gp a, x86::Gp b);
// x86_reg_t and Integral
template <typename T, typename = std::enable_if_t<std::is_integral<T>::value>>
x86_reg_t gen_operation(x86::Compiler& cc, comparison_operation op, x86_reg_t _a, T b);

/*
Type unary_operation
*/
x86_reg_t gen_operation(x86::Compiler& cc, unary_operation op, x86_reg_t _a);

// Inline functions
inline void mov(x86::Compiler& cc, x86_reg_t dest, x86::Mem source) {
    if(nonstd::holds_alternative<x86::Gp>(dest)) {
        cc.mov(nonstd::get<x86::Gp>(dest), source);
    } else {
        throw std::runtime_error("Variant not implemented in mov (Mem)");
    }
}
inline void mov(x86::Compiler& cc, x86::Mem dest, x86_reg_t source) {
    if(nonstd::holds_alternative<x86::Gp>(source)) {
        cc.mov(dest, nonstd::get<x86::Gp>(source));
    } else {
        throw std::runtime_error("Variant not implemented in mov (Mem as dest)");
    }
}

inline void mov(x86::Compiler& cc, x86::Mem dest, x86::Gp source) { cc.mov(dest, source); }

inline void cmp(x86::Compiler& cc, x86_reg_t a, x86_reg_t b) {
    if(nonstd::holds_alternative<x86::Gp>(a) && nonstd::holds_alternative<x86::Gp>(b)) {
        cc.cmp(nonstd::get<x86::Gp>(a), nonstd::get<x86::Gp>(b));
    } else {
        throw std::runtime_error("Variant not implemented in cmp");
    }
}

inline void cmp(x86::Compiler& cc, x86_reg_t a, x86::Mem b) {
    if(nonstd::holds_alternative<x86::Gp>(a)) {
        cc.cmp(nonstd::get<x86::Gp>(a), b);
    } else {
        throw std::runtime_error("Variant not implemented in cmp (Mem)");
    }
}
inline void cmp(x86::Compiler& cc, x86::Mem a, x86_reg_t b) {
    if(nonstd::holds_alternative<x86::Gp>(b)) {
        cc.cmp(a, nonstd::get<x86::Gp>(b));
    } else {
        throw std::runtime_error("Variant not implemented in cmp (Mem as dest)");
    }
}
inline x86::Gp get_reg_Gp(x86::Compiler& cc, unsigned size, bool is_signed) {
    assert(size <= 64);
    return nonstd::get<x86::Gp>(get_reg(cc, size, is_signed));
}
inline dGp get_reg_dGp(x86::Compiler& cc, unsigned size, bool is_signed) {
    assert(size == 128);
    return nonstd::get<dGp>(get_reg(cc, size, is_signed));
}
inline x86::Gp gen_ext_Gp(x86::Compiler& cc, x86_reg_t _val, unsigned size, bool is_signed) {
    assert(size <= 64);
    return nonstd::get<x86::Gp>(gen_ext(cc, _val, size, is_signed));
}
inline dGp gen_ext_dGp(x86::Compiler& cc, x86_reg_t _val, unsigned size, bool is_signed) {
    assert(size == 128);
    return nonstd::get<dGp>(gen_ext(cc, _val, size, is_signed));
}

// Templates
template <typename T, typename> inline void mov(x86::Compiler& cc, x86_reg_t dest, T source) {
    if(nonstd::holds_alternative<x86::Gp>(dest)) {
        cc.mov(nonstd::get<x86::Gp>(dest), source);
    } else {
        throw std::runtime_error("Variant not implemented in mov (Integral)");
    }
}
template <typename T, typename> inline void mov(x86::Compiler& cc, x86::Mem dest, T source) {
    // immediates larger than 32 bits need to be moved into a register first
    if(sizeof(T) <= 4) {
        cc.mov(dest, source);
    } else {
        x86::Gp imm_reg = get_reg_Gp(cc, sizeof(T) * 8);
        cc.mov(imm_reg, source);
        mov(cc, dest, imm_reg);
    }
}
template <typename T, typename> void cmp(x86::Compiler& cc, x86_reg_t _a, T b) {
    if(nonstd::holds_alternative<x86::Gp>(_a)) {
        x86::Gp a = nonstd::get<x86::Gp>(_a);
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
template <typename T, typename> inline void cmp(x86::Compiler& cc, x86::Mem a, T b) { cc.cmp(a, b); }

template <typename T, typename> inline x86_reg_t gen_ext(x86::Compiler& cc, T val, unsigned target_size, bool is_signed) {
    auto val_reg = get_reg(cc, sizeof(val) * 8, is_signed);
    mov(cc, val_reg, val);
    return gen_ext(cc, val_reg, target_size, is_signed);
}
template <typename T, typename> x86_reg_t gen_operation(x86::Compiler& cc, operation op, x86_reg_t _a, T b) {
    if(nonstd::holds_alternative<x86::Gp>(_a)) {
        x86::Gp a = nonstd::get<x86::Gp>(_a);
        return gen_operation_Gp(cc, op, a, b);
    } else if(nonstd::holds_alternative<dGp>(_a)) {
        dGp a = nonstd::get<dGp>(_a);
        dGp ret_val = dGp(cc);
        switch(op) {
        case add:
        case sub:
        case band:
        case bor:
        case bxor: {
            ret_val.upper = gen_operation_Gp(cc, op, a.upper, b >> 64);
            ret_val.lower = gen_operation_Gp(cc, op, a.lower, b & 0xffffffff);
            return ret_val;
        }
        case shl: {
            if(b >= 64) { // shl gets masked to 8 bits, so we need to shift by more than 63 manually
                assert(b <= 128);
                cc.mov(ret_val.lower, 0);
                ret_val.upper = gen_operation_Gp(cc, shl, a.lower, b - 64);
                return ret_val;
            } else {
                x86::Gp upper_shifted = gen_operation_Gp(cc, shl, a.upper, b);
                x86::Gp shifted_out_of_lower = gen_operation_Gp(cc, shr, a.lower, 64 - b);
                cc.or_(upper_shifted, shifted_out_of_lower);
                ret_val.upper = upper_shifted;
                ret_val.lower = gen_operation_Gp(cc, shl, a.lower, b);
                return ret_val;
            }
        }
        case sar: {
            if(b >= 64) { // sar gets masked to 8 bits, so we need to shift by more than 63 manually
                assert(b < 128);
                cc.mov(ret_val.upper, a.upper);
                cc.sar(ret_val.upper, 63);
                ret_val.lower = gen_operation_Gp(cc, sar, a.upper, b - 64);
                return ret_val;
            } else {
                x86::Gp lower_shifted = gen_operation_Gp(cc, shr, a.upper, b);
                x86::Gp shifted_out_of_upper = gen_operation_Gp(cc, shl, a.upper, 64 - b);
                cc.or_(lower_shifted, shifted_out_of_upper);
                ret_val.lower = lower_shifted;
                ret_val.upper = gen_operation_Gp(cc, sar, a.upper, b);
                return ret_val;
            }
        }
        case shr: {
            if(b >= 64) { // shr gets masked to 8 bits, so we need to shift by more than 63 manually
                assert(b < 128);
                cc.mov(ret_val.upper, 0);
                ret_val.lower = gen_operation_Gp(cc, shr, a.upper, b - 64);
                return ret_val;
            } else {
                x86::Gp lower_shifted = gen_operation_Gp(cc, shr, a.upper, b);
                x86::Gp shifted_out_of_upper = gen_operation_Gp(cc, shl, a.upper, 64 - b);
                cc.or_(lower_shifted, shifted_out_of_upper);
                ret_val.lower = lower_shifted;
                ret_val.upper = gen_operation_Gp(cc, shr, a.upper, b);
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
template <typename T, typename> x86::Gp gen_operation_Gp(x86::Compiler& cc, operation op, x86::Gp a, T b) {
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
template <typename V, typename W, typename> x86_reg_t gen_operation(x86::Compiler& cc, complex_operation op, V a, W b) {
    x86_reg_t a_reg = get_reg(cc, sizeof(a) * 8);
    x86_reg_t b_reg = get_reg(cc, sizeof(b) * 8);
    mov(cc, a_reg, a);
    mov(cc, b_reg, b);
    return gen_operation(cc, op, a_reg, b_reg);
}
template <typename T, typename> x86_reg_t gen_operation(x86::Compiler& cc, complex_operation op, x86_reg_t a, T b) {
    auto size = 0;
    if(nonstd::holds_alternative<x86::Gp>(a))
        size = nonstd::get<x86::Gp>(a).size() * 8;
    else if(nonstd::holds_alternative<dGp>(a))
        size = 128;
    else
        throw std::runtime_error("Invalid variant in gen_operation");
    x86_reg_t b_reg = get_reg(cc, size);
    mov(cc, b_reg, b);
    return gen_operation(cc, op, a, b_reg);
}
template <typename T, typename> x86_reg_t gen_operation(x86::Compiler& cc, comparison_operation op, x86_reg_t _a, T b) {
    if(nonstd::holds_alternative<x86::Gp>(_a)) {
        x86::Gp a = nonstd::get<x86::Gp>(_a);
        return gen_operation_Gp(cc, op, a, b);
    } else {
        throw std::runtime_error("Variant not supported in gen_operation (comparison)");
    }
}
template <typename T, typename> x86::Gp gen_operation_Gp(x86::Compiler& cc, comparison_operation op, x86::Gp a, T b) {
    x86::Gp b_reg = get_reg_Gp(cc, sizeof(T) * 8);
    cc.mov(b_reg, b);
    x86::Gp big_b = gen_ext_Gp(cc, b_reg, a.size() * 8, std::is_signed<T>::value);
    return gen_operation_Gp(cc, op, a, big_b);
}
} // namespace asmjit
} // namespace iss
#endif /* ASMJIT_VM_UTIL_H_ */
