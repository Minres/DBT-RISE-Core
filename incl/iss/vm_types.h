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

#ifndef _VM_TYPES_H_
#define _VM_TYPES_H_

#include <cassert>
#include <cstdint>
#include <iostream>
#include <limits>

namespace iss {

enum status { Ok, Err, NotSupported };

enum sync_type { NO_SYNC = 0U, PRE_SYNC = 1U, POST_SYNC = 2U, ALL_SYNC = 3U };

inline sync_type operator |(sync_type a, sync_type b) {
    return static_cast<sync_type>(static_cast<int>(a) | static_cast<int>(b));
}

inline sync_type operator &(sync_type a, sync_type b) {
    return static_cast<sync_type>(static_cast<int>(a) & static_cast<int>(b));
}

inline sync_type& operator |=(sync_type& a, sync_type b){ return a = a | b; }

enum struct access_type : uint16_t {
    // operations
    READ = 0x0,
    WRITE = 0x1,
    FETCH = 0x2,
    DEBUG_READ = 0x4,
    DEBUG_WRITE = 0x5,
    DEBUG_FETCH = 0x6,
    // bit meanings
    READ_WRITE = 0x1,
    CODE_DATA = 0x2,
    FUNC = 0x3,
    DEBUG = 0x4,
    // aliases
    DATA = 0x0,
    CODE = 0x2
};

inline access_type operator&(access_type a1, access_type a2) {
    return static_cast<access_type>(static_cast<uint16_t>(a1) & static_cast<uint16_t>(a2));
}

inline access_type operator&(access_type a1, uint16_t a2) {
    return static_cast<access_type>(static_cast<uint16_t>(a1) & a2);
}

inline bool operator==(access_type a, uint16_t i) { return static_cast<uint16_t>(a) == i; }

inline bool operator!=(access_type a, uint16_t i) { return static_cast<uint16_t>(a) != i; }

inline bool operator&&(access_type a1, access_type a2) {
    return (static_cast<uint16_t>(a1) & static_cast<uint16_t>(a2));
}

enum struct address_type : uint16_t { LOGICAL, VIRTUAL, PHYSICAL };

class addr_t {
public:
    const address_type type = address_type::LOGICAL;
    const access_type access = access_type::WRITE;
    uint32_t space = 0;
    uint64_t val = 0;

    constexpr addr_t(access_type acc_type, address_type addr_type, uint32_t space, uint64_t addr)
    : type(addr_type)
    , access(acc_type)
    , space(space)
    , val(addr) {}

    const addr_t &operator=(uint64_t o) {
        val = o;
        return *this;
    }

    const addr_t &operator=(const addr_t &o) {
        val = o.val;
        return *this;
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

    addr_t &operator&=(uint64_t m) {
        val &= m;
        return *this;
    }
};

inline addr_t operator+(const addr_t &a, const addr_t &o) {
    assert(a.type == o.type && a.space == o.space);
    return addr_t{a.access, a.type, a.space, a.val + o.val};
}

inline addr_t operator-(const addr_t &a, const addr_t &o) {
    assert(a.type == o.type && a.space == o.space);
    return addr_t{a.access, a.type, a.space, a.val - o.val};
}

inline addr_t operator+(addr_t &a, uint64_t m) { return addr_t{a.access, a.type, a.space, a.val + m}; }

inline addr_t operator-(addr_t &a, uint64_t m) { return addr_t{a.access, a.type, a.space, a.val - m}; }

inline std::ostream &operator<<(std::ostream &os, const addr_t &op) {
    os << "[" << op.space << "]0x" << std::hex << op.val << std::dec;
    return os;
}

template <address_type TYPE> class typed_addr_t : public addr_t {
public:
    constexpr typed_addr_t()
    : addr_t(access_type::WRITE, TYPE, 0, 0){};
    constexpr typed_addr_t(access_type acc_type, uint32_t space, uint64_t v)
    : addr_t(acc_type, TYPE, space, v) {}
    constexpr typed_addr_t(access_type t, uint64_t v)
    : addr_t(t, TYPE, 0, v) {}
    constexpr typed_addr_t(const addr_t &o)
    : typed_addr_t(o.access, o.space, o.val) {}
};
}

namespace iss {
template <typename T, int ARCH> class PrimitiveTypeHolder {
    using this_type = PrimitiveTypeHolder<T, ARCH>;

public:
    explicit PrimitiveTypeHolder(T v)
    : v(v) {}
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
