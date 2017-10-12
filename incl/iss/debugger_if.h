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

#ifndef _DEBUGGER_IF_H_
#define _DEBUGGER_IF_H_

#include "debugger/server_if.h"
#include "debugger/target_adapter_if.h"
#include "vm_types.h"
#include <vector>

namespace iss {

class debugger_if {
public:
    /**
     * destructor
     */
    virtual ~debugger_if() = default;
    /**
     * check if debugging is enabled
     * @return
     */
    inline bool debugging_enabled() { return dbg_enabled; }
    /**
     * return a target adapter for accessing the core to be debugged
     * @param server server interface to be used for the target adapter
     * @return the target adapter for the core
     */
    virtual debugger::target_adapter_if *accquire_target_adapter(debugger::server_if *server) = 0;

protected:
    bool dbg_enabled = false;
};
}

#endif /* _DEBUGGER_IF_H_ */
