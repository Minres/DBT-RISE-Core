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

#ifndef _INCL_ISS_INSTRUMENTATION_IF_H_
#define _INCL_ISS_INSTRUMENTATION_IF_H_

#include <cstdint>
#include <string>

namespace iss {

namespace v1 {
struct instrumentation_if {

    virtual ~instrumentation_if(){};

    /**
     * get the name of this architecture
     *
     * @return the name of this architecture
     */
    virtual const std::string core_type_name() const = 0;
    /**
     * Retrieve the current value of the program counter
     *
     * @return the value of the PC
     */
    virtual uint64_t get_pc() = 0;
    /**
     * Retrieve the current value of the program counter of the next instruction
     *
     * @return the value of the next PC
     */
    virtual uint64_t get_next_pc() = 0;
    /**
     * Retrieve the current instruction word being processed
     *
     * @return the value of the next PC
     */
    virtual uint64_t get_instr_word() = 0;
    /**
     * Retrieve the current value of the program counter of the next instruction
     *
     * @return the binary value of the current instruction
     */
    virtual uint64_t get_instr_count() = 0;
    /**
     * Retrieve the pending traps of the ISS if there are any
     *
     * @return the pending traps
     */
    virtual uint64_t get_pendig_traps() = 0;
    /**
     * Retrieve the current value of the program counter of the next instruction
     *
     * @return the value of the next PC
     */
    virtual uint64_t get_total_cycles() = 0;
    /**
     * update the cycle count (default is 1) of the last executed instruction
     *
     * @param cycles
     */
    virtual void set_curr_instr_cycles(unsigned cycles) = 0;
};
} // namespace v1
inline namespace v2 {
struct instrumentation_if {

    virtual ~instrumentation_if(){};

    /**
     * get the name of this architecture
     *
     * @return the name of this architecture
     */
    virtual const std::string core_type_name() const = 0;
    /**
     * Retrieve the current value of the program counter
     *
     * @return the value of the PC
     */
    virtual uint64_t get_pc() = 0;
    /**
     * Retrieve the current value of the program counter of the next instruction
     *
     * @return the value of the next PC
     */
    virtual uint64_t get_next_pc() = 0;
    /**
     * Retrieve the current instruction word being processed
     *
     * @return the value of the next PC
     */
    virtual uint64_t get_instr_word() = 0;
    /**
     * Retrieve the current value of the program counter of the next instruction
     *
     * @return the binary value of the current instruction
     */
    virtual uint64_t get_instr_count() = 0;
    /**
     * Retrieve the pending traps of the ISS if there are any
     *
     * @return the pending traps
     */
    virtual uint64_t get_pendig_traps() = 0;
    /**
     * Retrieve the total number of cycles executed by the core
     *
     * @return number of cycles
     */
    virtual uint64_t get_total_cycles() = 0;
    /**
     * update the cycle count (default is 1) of the last executed instruction
     *
     * @param cycles
     */
    virtual void update_last_instr_cycles(unsigned cycles) = 0;
    /**
     * indicates if the last branch was taken or not
     *
     * @return true if branch was taken
     */
    virtual bool is_branch_taken() = 0;
    /**
     * return the number of registers used by the core
     *
     * @return number of registers
     */
    virtual unsigned get_reg_num() = 0;
    /**
     * return the size of requested registers
     *
     * @param num number of register
     *
     * @return size of register in bits
     */
    virtual unsigned get_reg_size(unsigned) = 0;
};
} // namespace v2
} /* namespace iss */

#endif /* _INCL_ISS_INSTRUMENTATION_IF_H_ */
