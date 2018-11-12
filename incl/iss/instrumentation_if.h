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

#ifndef _INCL_ISS_INSTRUMENTATION_IF_H_
#define _INCL_ISS_INSTRUMENTATION_IF_H_

#include <cstdint>
#include <string>

namespace iss {

inline namespace v1 {}

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
     * update the cycle count (default is 1) of the last executed instruction
     *
     * @param cycles
     */
    virtual void set_curr_instr_cycles(unsigned cycles) = 0;
};
}

} /* namespace iss */

#endif /* _INCL_ISS_INSTRUMENTATION_IF_H_ */
