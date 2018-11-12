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
 * Contributors:
 *       eyck@minres.com - initial API and implementation
 ******************************************************************************/

// switch off warning C4996: 'std::copy': Function call with parameters that may
// be unsafe
#include "iss/debugger/server_base.h"
#include <functional>
#include <future>
#include <iss/debugger_if.h>
#include <iss/vm_if.h>

using namespace iss::debugger;
using namespace logging;

template <typename T> static T to(unsigned char *data, size_t num_bytes) {
    T res = 0;
    for (unsigned i = 0; i < num_bytes; i++) res += (T)data[i] << +i * 8;
    return res;
}

server_base::server_base(iss::debugger_if *vm)
: vm(vm)
, tgt(nullptr) {}

int server_base::dummy_func() { return 42; }
// called from debugger
void server_base::step(unsigned coreId, unsigned steps) {
    cycles.store(steps, std::memory_order_relaxed);
    mode.store(MODE_RUN, std::memory_order_release);
    syncronizer.enqueue_and_wait(&server_base::dummy_func, this);
    while (!syncronizer.is_ready()) std::this_thread::sleep_for(std::chrono::milliseconds(10));
    LOG(TRACE) << "step finished";
}
// called from debugger
void server_base::run(unsigned coreId) {
    if (mode == MODE_RUN) return;
    cycles.store(std::numeric_limits<uint64_t>::max(), std::memory_order_relaxed);
    mode.store(MODE_RUN, std::memory_order_release);
    syncronizer.enqueue_and_wait(&server_base::dummy_func, this);
}
void server_base::run(unsigned coreId, std::function<void(unsigned)> callback) {
    if (mode == MODE_RUN) return;
    cycles.store(std::numeric_limits<uint64_t>::max(), std::memory_order_relaxed);
    mode.store(MODE_RUN, std::memory_order_release);
    this->stop_callback = callback;
    syncronizer.enqueue(&server_base::dummy_func, this);
}
// called from debugger
void server_base::request_stop(unsigned coreId) { mode.store(MODE_STOP, std::memory_order_relaxed); }

void server_base::wait_for_stop() {
    // busy wait
    while (mode != MODE_STOP) std::this_thread::sleep_for(std::chrono::milliseconds(10));
}

iss::status server_base::reset(int coreId) { return iss::Ok; }
