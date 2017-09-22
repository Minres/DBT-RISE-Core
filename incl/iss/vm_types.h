/*******************************************************************************
 * Copyright (C) 2017, MINRES Technologies GmbH
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

#ifndef _VM_TYPES_H_
#define _VM_TYPES_H_

#include <cassert>
#include <cstdint>
#include <iostream>
#include <limits>

namespace iss {

enum status { Ok, Err, NotSupported };

enum sync_type { NO_SYNC = 0, PRE_SYNC = 1, POST_SYNC = 2, ALL_SYNC = 3 };

enum access_type {
    WRITE = 0,
    READ = 1,
    FETCH = 3,
    DATA = 0,
    CODE = 2,
    DEBUG_WRITE = 4,
    DEBUG_READ = 5,
    DEBUG_FETCH = 7,
    DEBUG = DEBUG_WRITE,
    ACCESS_TYPE = 0x7
};

enum address_type { LOGICAL = 0x10, VIRTUAL = 0x20, PHYSICAL = 0x30, ADDRESS_TYPE = 0x30 };

struct addr_t {
    const unsigned type;
    unsigned space=0;
    uint64_t val;

    unsigned getAccessType() const { return type & ACCESS_TYPE; };

    constexpr addr_t() : type(READ | PHYSICAL), val(0) {}
    constexpr addr_t(access_type acc_t, address_type addr_t, unsigned s, uint64_t addr)
        : type(acc_t | addr_t), space(s), val(addr) {}
    constexpr addr_t(unsigned t, unsigned s, uint64_t addr) : type(t), space(s), val(addr) {}
    constexpr addr_t(const addr_t &o) : type(o.type), space(o.space), val(o.val) {}

    constexpr addr_t &operator=(uint64_t o) {
        val = o;
        return *this;
    }

    constexpr addr_t &operator=(const addr_t &o) {
        val = o.val;
        return *this;
    }

    constexpr addr_t operator+(const addr_t &o) const {
        assert(type == o.type && space == o.space);
        addr_t ret(*this);
        ret.val += o.val;
        return ret;
    }

    addr_t &operator++() { // prefix increment
        val++;
        return *this;
    }

    addr_t operator++(int unused) { // postfix increment
        addr_t ret(*this);
        val++;
        return ret;
    }

    addr_t &operator--() {
        val--;
        return *this;
    }

    constexpr addr_t operator+(int m) const {
        addr_t ret(*this);
        ret.val += m;
        return ret;
    }

    constexpr addr_t operator-(int m) const {
        addr_t ret(*this);
        ret.val -= m;
        return ret;
    }
};

inline std::ostream &operator<<(std::ostream &os, const addr_t &op) {
    os << "[" << op.space << "]0x" << std::hex << op.val << std::dec;
    return os;
}

template <address_type TYPE> struct typed_addr_t : public addr_t {
    constexpr typed_addr_t() : addr_t(READ, TYPE, 0) = default;
    constexpr typed_addr_t(access_type t, uint64_t v) : addr_t(t, TYPE, 0, v) {}
    constexpr typed_addr_t(unsigned t, unsigned s, uint64_t v) : addr_t((t & ACCESS_TYPE) | TYPE, s, v) {}
    constexpr typed_addr_t(const addr_t &o) : addr_t(o.getAccessType() | TYPE, o.space, o.val) {}
};
}

namespace iss {
template <typename T, int ARCH> class PrimitiveTypeHolder {
    using  this_type = PrimitiveTypeHolder<T, ARCH> ;

public:
    explicit PrimitiveTypeHolder(T v) : v(v) {}
    operator const T() const { return v; }
    const this_type operator+(const this_type &other) const { return this_type(v + other.v); }
    this_type &operator++() { // prefix increment
        v++;
        return *this;
    }
    this_type operator++(int unused) { // postfix increment
        this_type ret(*this);
        v++;
        return ret;
    }
    this_type &operator--() {
        v--;
        return *this;
    }
    this_type operator+(int m) const { return this_type(v + m); }

    this_type operator-(int m) const { return this_type(v - m); }

    this_type &operator=(T o) {
        v = o;
        return *this;
    }

private:
    T v;
};
}

#endif /* _VM_TYPES_H_ */
