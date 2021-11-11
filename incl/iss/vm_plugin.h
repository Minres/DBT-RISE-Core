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

#ifndef _ISS_VM_PLUGIN_H_
#define _ISS_VM_PLUGIN_H_

#include "util/bit_field.h"
#include "vm_if.h"
#include <memory>

namespace iss {

BEGIN_BF_DECL(instr_info_t, uint64_t)
BF_FIELD(cluster_id, 56, 8)
BF_FIELD(core_id, 40, 16)
BF_FIELD(instr_id, 24, 16)
BF_FIELD(phase_id, 16, 8)
instr_info_t(unsigned cluster_id, unsigned core_id, unsigned instr_id, unsigned phase_id)
: instr_info_t() {
    this->cluster_id = cluster_id & std::numeric_limits<uint8_t>::max();
    this->core_id = core_id & std::numeric_limits<uint16_t>::max();
    this->instr_id = instr_id & std::numeric_limits<uint16_t>::max();
    this->phase_id = phase_id & std::numeric_limits<uint8_t>::max();
}
END_BF_DECL();

struct exec_info {
    bool branch_taken{false};
};
class vm_plugin { // @suppress("Class has a virtual method and non-virtual destructor")
public:
    virtual ~vm_plugin() {}

    virtual bool registration(const char *const version, vm_if &arch) = 0;

    virtual sync_type get_sync() = 0;

    virtual void callback(instr_info_t instr_info, exec_info const&) = 0;
};
}

#endif /* DBT_CORE_INCL_ISS_VM_PLUGIN_H_ */
