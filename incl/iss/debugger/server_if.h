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

#ifndef _SERVER_IF_H_
#define _SERVER_IF_H_

#include <iss/vm_if.h>
#include <util/thread_syncronizer.h>

#include <atomic>
#include <deque>
#include <string>
#include <type_traits>
#include <vector>

namespace iss {

class debugger_if;

namespace debugger {
// forward declaration
class target_adapter_if;
/**
 * the debug server interface
 */
class server_if {
public:
    friend class iss::debugger_if;
    /**
     * debugger access type
     */
    enum access_type { Read, Write };
    /**
     * type of breakpoints
     */
    enum bp_type { OnExec = 0, OnRead = 1, OnWrite = 2, OnRW = 3 };
    /**
     * address type
     */
    enum addr_type { Phys, Virt, Log };
    /**
     * execution mode of debuggee
     */
    enum mode_e { MODE_RUN, MODE_BREAK, MODE_STOP };

    /**
     * destructor
     */
    virtual ~server_if() = default;
    /**
     * run the specified core, numeric_limits<unsigned>::max() indicates all cores
     * @param coreId the core to let run
     */
    virtual void run(unsigned coreId) = 0;
    /**
     * run the specified core asynchronously, numeric_limits<unsigned>::max() indicates all cores
     * @param coreId the core to let run
     * @param callback to be called when run stops
     */
    virtual void run(unsigned coreId, std::function<void(unsigned)> callback) = 0;
    /**
     * stop the specified core (non-blocking)
     * @param coreId the core to stop
     */
    virtual void request_stop(unsigned coreId) = 0;
    /**
     * check if a core is executing
     * @return true if the core runs
     */
    virtual bool is_running() { return mode.load() != MODE_STOP; }
    /**
     * wait until request_stop() is processed and the specified core stops
     */
    virtual void wait_for_stop() = 0;
    /**
     * single step the specified core
     * @param coreId the core to single step
     * @param steps number of steps to execute
     */
    virtual void step(unsigned coreId, unsigned steps = 1) = 0;
    /**
     * reset the core and system
     * @param coreId core to reset
     * @return result of the operation
     */
    virtual status reset(int coreId) = 0;
    /**
     * shut down the simulation and server
     */
    virtual void shutdown() = 0;

    enum exec_states { initialized = 0, running = 1, stopped = 2, stepping = 3 };
    /**
     * get the target adapter for the core being debugged
     * @return
     */
    virtual target_adapter_if *get_target() = 0;
    /**
     * check if the simulation can continue
     * @param bp_handle the handle of breakpoint condition being met, 0 means no
     * hit
     */
    inline void check_continue(unsigned bp_handle) {
        if (bp_handle) {
            mode.store(MODE_STOP, std::memory_order_release);
            last_bp = bp_handle;
        }
        if (mode.load(std::memory_order_acquire) == MODE_STOP) {
            if (stop_callback) {
                stop_callback(last_bp);
                stop_callback = std::function<void(unsigned)>();
            }
            while (mode.load(std::memory_order_acquire) == MODE_STOP) {
                syncronizer.executeNext();
            }
            last_bp = 0;
        }
        if (--cycles == 0) mode = MODE_STOP;
    }
    /**
     * execute the specified function synchronized in the simulation thread
     * @param f function to execute
     * @param args arguments of the function
     * @return result value of function (if any)
     */
    template <class F, class... Args>
    typename std::result_of<F(Args...)>::type execute_syncronized(F &&f, Args &&... args) {
        return syncronizer.enqueue_and_wait(f, args...);
    }

protected:
    util::thread_syncronizer syncronizer;
    std::atomic<mode_e> mode{MODE_STOP};
    std::atomic<uint64_t> cycles;
    unsigned last_bp;
    std::function<void(unsigned)> stop_callback;
};

} // namespace debugger
} // namspace iss

#endif /* _SERVER_IF_H_ */
