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

#ifndef _ARCH_IF_H_
#define _ARCH_IF_H_

#include <dbt_rise_common.h>
#include "vm_types.h"

#include <algorithm>
#include <iomanip>
#include <iostream>
#include <vector>

namespace iss {
/**
 * exception to be thrown if the architecture encounters a synchronous trap
 */
class trap_access {
public:
    trap_access(unsigned id, uint64_t addr)
    : id(id)
    , addr(addr) {}
    const unsigned id;
    const uint64_t addr;
};

inline namespace v1 { struct instrumentation_if; }
/**
 * architecture interface
 */
class arch_if {
public:
    /* deprecated */
    enum operations {
        NOP,
        AND,
        OR,
        XOR,
        ADD,
        ADC,
        SUB,
        SUBC,
        MUL,
        DIV,
        ADDL,
        ADCL,
        SUBL,
        SUBCL,
        MULL,
        DIVL,
    };

    virtual ~arch_if() = default;
    /**
     * execution phases: instruction start and end
     */
    enum exec_phase { ISTART, IEND, BSTART, BEND };
    /**
     * reset the core
     *
     * @param address where to start from
     */
    virtual void reset(uint64_t address) = 0;
    /**
     * preload memories from file
     *
     * @param name name of th efile to load
     * @param type of file, implementation dependent
     */
    virtual std::pair<uint64_t, bool> load_file(std::string name, int type = -1) = 0;

    /**
     * notify the core about the execution phase (if needed)
     *
     * @param the actual phase the vm is in
     */
    virtual void notify_phase(exec_phase){};
    /**
     * get the pointer to the register array
     *
     * @return pointer to the registers
     */
    virtual uint8_t *get_regs_base_ptr() = 0;

    /**
     * read from addresses
     *
     * @param addr address to read from, contains access type, address space and
     * address
     * @param length length of the data to read
     * @param data pointer to the memory to read into
     * @return success or failure of access
     */
    inline iss::status read(const addr_t &addr, const unsigned length, uint8_t *const data){
        return read(addr.type, addr.access, addr.space, addr.val, length, data);
    }
    /**
     * read from addresses
     *
     * @param addr address to read from, contains access type, address space and
     * address
     * @param length length of the data to read
     * @param data pointer to the memory to read into
     * @return success or failure of access
     */
    virtual iss::status read(const address_type type, const access_type access, const uint32_t space,
            const uint64_t addr, const unsigned length, uint8_t *const data) = 0;
    /**
     * write to addresses
     *
     * @param addr address to read from, contains access type, address space and
     * address
     * @param length length of the data to write
     * @param data pointer to the memory to write from
     * @return success or failure of access
     */
    inline iss::status write(const addr_t &addr, const unsigned length, const uint8_t *const data){
        return write(addr.type, addr.access, addr.space, addr.val, length, data);
    }
    /**
     * write to addresses
     *
     * @param addr address to read from, contains access type, address space and
     * address
     * @param length length of the data to write
     * @param data pointer to the memory to write from
     * @return success or failure of access
     */
    virtual iss::status write(const address_type type, const access_type access, const uint32_t space,
            const uint64_t addr, const unsigned length, const uint8_t *const data) = 0;
   /**
     * vm encountered a trap (exception, interrupt), process accordingly in core
     *
     * @param flags trap flags
     * @return new (virtual) address to continue from
     */
    virtual uint64_t enter_trap(uint64_t flags) { return 0; }
    /**
     * vm encountered a trap (exception, interrupt), process accordingly in core
     *
     * @param flags trap flags
     * @param addr address where the trap enountered
     * @return new (virtual) address to continue from
     */
    virtual uint64_t enter_trap(uint64_t flags, uint64_t addr, uint64_t instr) { return 0; }
    /**
     * vm decoded the instruction to return from trap (exception, interrupt),
     * process accordingly in core
     *
     * @param flags trap flags
     * @return new (virtual) address to continue from
     */
    virtual uint64_t leave_trap(uint64_t flags) { return 0; }
    /**
     * wait until condition indicated by flags becomes true
     *
     * @param flags indicating the condition
     */
    virtual void wait_until(uint64_t flags) {}
    /**
     * retrieve information to augment the disassembly
     *
     * @return string containing the core status in text form
     */
    virtual void disass_output(uint64_t pc, const std::string instr) {
        std::cout << "0x" << std::setw(16) << std::setfill('0') << std::hex << pc << "\t\t" << instr << std::endl;
    };
    /**
     * get the pointer to the instrumentation interface. In case there is no instrumentation
     * supported a null pointer is returned
     *
     * @return non-owning pointer to the instrumentation interface of the architecture or nullptr
     */
    virtual v1::instrumentation_if *get_instrumentation_if() { return nullptr; };
};
}

#endif /* _ARCH_IF_H_ */
