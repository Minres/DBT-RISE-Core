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

#ifndef _TARGET_ADAPTER_BASE_H_
#define _TARGET_ADAPTER_BASE_H_

#include "target_adapter_if.h"
#include <iss/debugger/server_if.h>
#include <iss/vm_types.h>
#include <util/range_lut.h>

namespace iss {
namespace debugger {

class target_adapter_base : public target_adapter_if {
public:
    target_adapter_base(iss::debugger::server_if *srv)
    : srv(srv)
    , bp_lut(0) {}

    void set_server(iss::debugger::server_if *server) { srv = server; }

    inline void check_continue(uint64_t pc) {
        unsigned handle = bp_lut.getEntry(pc);
        if (!handle && break_cond) {
            handle = break_cond();
            if (handle) break_cond = std::function<unsigned()>();
        }
        srv->check_continue(handle);
    }

    /* return table of remote commands */
    const std::vector<target_adapter_if::custom_command> &custom_commands() override { return ccmds; }

    void add_custom_command(custom_command &&cmd) override { ccmds.push_back(cmd); };

    void help(const char *prog_name) override;

    iss::status open(int argc, char *const agrv[], const char *prog_name, log_func log_fn) override;

    void close() override;

    iss::status connect(bool &can_restart) override;

    iss::status disconnect() override;

    void kill() override;

    iss::status restart() override;

    void stop() override;

    iss::status resume_from_current(bool step, int sig, rp_thread_ref thread,
                                    std::function<void(unsigned)> stop_callback) override;

    iss::status wait_non_blocking(bool &running) override;

    iss::status wait_blocking() override;

    iss::status add_break_condition(std::function<unsigned()> break_cond) override;

protected:
    iss::debugger::server_if *srv;
    util::range_lut<unsigned> bp_lut;
    long bp_count = 0;
    std::function<unsigned()> break_cond;
    std::vector<target_adapter_if::custom_command> ccmds;
};

} // namespace debugger
} // namspace iss

#endif /* _TARGETADAPTER_H_ */
