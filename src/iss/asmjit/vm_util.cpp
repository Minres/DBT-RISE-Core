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

#include <asmjit/asmjit.h>
#include <cassert>
#include <fmt/core.h>
#include <fmt/format.h>
#include <iss/asmjit/vm_util.h>

namespace iss {

namespace asmjit {
using namespace ::asmjit;

void mov(x86::Compiler& cc, x86_reg_t _dest, x86_reg_t _source) {
    if(nonstd::holds_alternative<x86::Gp>(_dest) && nonstd::holds_alternative<x86::Gp>(_source)) {
        x86::Gp dest = nonstd::get<x86::Gp>(_dest);
        x86::Gp source = nonstd::get<x86::Gp>(_source);
        assert(dest.size() == source.size());
        cc.mov(dest, source);
    } else {
        throw std::runtime_error("Variant not implemented in mov");
    }
}

x86_reg_t get_reg(x86::Compiler& cc, unsigned size, bool is_signed) {
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

x86_reg_t gen_ext(x86::Compiler& cc, x86_reg_t _val, unsigned target_size, bool is_signed) {
    // In case of x86::Gp
    if(nonstd::holds_alternative<x86::Gp>(_val)) {
        auto val = nonstd::get<x86::Gp>(_val);
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
    else if(nonstd::holds_alternative<dGp>(_val)) {
        auto val = nonstd::get<dGp>(_val);
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

x86_reg_t gen_operation(x86::Compiler& cc, operation op, x86_reg_t _a, x86_reg_t _b) {
    if(nonstd::holds_alternative<x86::Gp>(_a) && nonstd::holds_alternative<x86::Gp>(_b)) {
        x86::Gp a = nonstd::get<x86::Gp>(_a);
        x86::Gp b = nonstd::get<x86::Gp>(_b);
        assert(a.size() == b.size());
        return gen_operation_Gp(cc, op, a, b);
    }
    // Should not end here
    else {
        throw std::runtime_error("Invalid variant combination in in gen_operation (operation)");
        return _a;
    }
}

void expand_division_operand(x86::Compiler& cc, x86::Gp upper_half, x86::Gp dividend) {
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

x86_reg_t gen_operation(x86::Compiler& cc, complex_operation op, x86_reg_t _a, x86_reg_t _b) {
    if(nonstd::holds_alternative<x86::Gp>(_a) && nonstd::holds_alternative<x86::Gp>(_b)) {
        x86::Gp a = nonstd::get<x86::Gp>(_a);
        x86::Gp b = nonstd::get<x86::Gp>(_b);
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
    if(nonstd::holds_alternative<x86::Gp>(_a) && nonstd::holds_alternative<x86::Gp>(_b)) {
        x86::Gp a = nonstd::get<x86::Gp>(_a);
        x86::Gp b = nonstd::get<x86::Gp>(_b);
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
    } else if(nonstd::holds_alternative<dGp>(_a) && nonstd::holds_alternative<dGp>(_b)) {
        throw std::runtime_error("Multiplication with input operands >64-bit not yet implemented");
        return _b;
    }
    throw std::runtime_error("Variant combination not handled in gen_operation_Gp (complex)");
    return _b;
}

x86_reg_t gen_operation(x86::Compiler& cc, comparison_operation op, x86_reg_t _a, x86_reg_t _b) {
    if(nonstd::holds_alternative<x86::Gp>(_a) && nonstd::holds_alternative<x86::Gp>(_b)) {
        x86::Gp a = nonstd::get<x86::Gp>(_a);
        x86::Gp b = nonstd::get<x86::Gp>(_b);
        return gen_operation_Gp(cc, op, a, b);
    } else {
        throw std::runtime_error("Variant combination not supported in gen_operation (comparison)");
    }
}
x86::Gp gen_operation_Gp(x86::Compiler& cc, comparison_operation op, x86::Gp a, x86::Gp b) {
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

x86_reg_t gen_operation(x86::Compiler& cc, unary_operation op, x86_reg_t _a) {
    if(nonstd::holds_alternative<x86::Gp>(_a)) {
        x86::Gp a = nonstd::get<x86::Gp>(_a);
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

void setArg(InvokeNode* f_node, uint64_t argPos, x86_reg_t _arg) {
    if(nonstd::holds_alternative<x86::Gp>(_arg)) {
        x86::Gp arg = nonstd::get<x86::Gp>(_arg);
        setArg(f_node, argPos, arg);
    } else {
        throw std::runtime_error("Variant not supported in setArg");
    }
}

void setArg(InvokeNode* f_node, uint64_t argPos, x86::Gp arg) { f_node->setArg(argPos, arg); }

void setRet(InvokeNode* f_node, uint64_t argPos, x86_reg_t _arg) {
    if(nonstd::holds_alternative<x86::Gp>(_arg)) {
        x86::Gp arg = nonstd::get<x86::Gp>(_arg);
        setRet(f_node, argPos, arg);
    } else {
        throw std::runtime_error("Variant not supported in setRet");
    }
}

void setRet(InvokeNode* f_node, uint64_t argPos, x86::Gp arg) { f_node->setRet(argPos, arg); }
} // namespace asmjit
} // namespace iss
