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

#ifndef _SERVER_BASE_H_
#define _SERVER_BASE_H_

#include <iss/log_categories.h>

#include "server_if.h"
#include <iss/debugger_if.h>

namespace iss {
namespace debugger {

class server_base : public server_if {
public:
    server_base(debugger_if* adapter);

    unsigned add_debugger_adapter(iss::debugger_if* vm) {
        this->vm.push_back(vm);
        return this->vm.size() - 1;
    }

    unsigned int get_reg_width(int index) const;

    void run(unsigned coreId) override;

    void run(unsigned coreId, std::function<void(unsigned)> callback) override;

    void request_stop(unsigned coreId) override;

    void wait_for_stop() override;

    void step(unsigned coreId, unsigned steps = 1) override;

    iss::status reset(unsigned coreId) override;

    target_adapter_if* get_target(unsigned coreId) override { return vm[coreId]->accquire_target_adapter(this); }

protected:
    std::vector<debugger_if*> vm;
    // used to make sure that the other thread got invoked
    int dummy_func();
};

} // namespace debugger
} // namespace iss

#endif /* _SERVER_BASE_H_ */
