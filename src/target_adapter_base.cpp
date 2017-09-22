////////////////////////////////////////////////////////////////////////////////
// Copyright (C) 2017, MINRES Technologies GmbH
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//
// 1. Redistributions of source code must retain the above copyright notice,
//    this list of conditions and the following disclaimer.
//
// 2. Redistributions in binary form must reproduce the above copyright notice,
//    this list of conditions and the following disclaimer in the documentation
//    and/or other materials provided with the distribution.
//
// 3. Neither the name of the copyright holder nor the names of its contributors
//    may be used to endorse or promote products derived from this software
//    without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
// LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
// CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
// SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
// INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
// CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
// ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
//
// Contributors:
//       eyck@minres.com - initial API and implementation
////////////////////////////////////////////////////////////////////////////////

#include <iss/debugger/target_adapter_base.h>

using namespace iss::debugger;

void target_adapter_base::help(const char *prog_name) {}

iss::status target_adapter_base::open(int argc, char *const agrv[], const char *prog_name, log_func log_fn) {
    return iss::Ok;
}

void target_adapter_base::close() { bp_lut.clear(); }

iss::status target_adapter_base::connect(std::string &status_string, bool &can_restart) { return iss::Ok; }

iss::status target_adapter_base::disconnect() { return iss::Ok; }

void target_adapter_base::kill() {}

iss::status target_adapter_base::restart() { return iss::Ok; }

void target_adapter_base::stop() {
    srv->request_stop(0);
    srv->wait_for_stop();
}

iss::status target_adapter_base::resume_from_current(bool step, int sig) {
    if (step)
        srv->step(0, 1);
    else
        srv->run(0);
    return iss::Ok;
}

iss::status target_adapter_base::wait_non_blocking(std::string &status_string, out_func out, bool &running) {
    running = srv->is_running();
    if (running) {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        running = srv->is_running();
    }
    return iss::NotSupported;
}

iss::status target_adapter_base::wait_blocking(std::string &status_string, out_func out) {
    srv->wait_for_stop();
    return iss::Ok;
}
