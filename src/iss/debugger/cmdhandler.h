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

#ifndef _CMDHANDLER_H_
#define _CMDHANDLER_H_

#include "encoderdecoder.h"
#include "server_if.h"
#include "target_adapter_if.h"

#include <array>
#include <boost/optional.hpp>
#include <unordered_map>

namespace iss {
namespace debugger {

enum {
    MAX_DATABYTES = 0x10000, /* Size of data buffer  */
    INOUTBUF_SIZE = 2 * MAX_DATABYTES + 32,
    TARGET_SIGNAL0 = 0,
    DBG_PRINTBUF_SIZE = 400,
    MAXARGS = 4
};

enum signals {
    /* Signal types */
    HANGUP = 1,
    INTERRUPT = 2,
    QUIT = 3,
    ILLEGAL_INSTRUCTION = 4,
    BREAKPOINT = 5,
    ABORTED = 6,
    BUS_ERROR = 7,
    FLOATING_POINT_EXCEPTION = 8,
    KILLED = 9,
    SEGMENT_VIOLATION = 11,
    TERMINATED = 15,
    STK_FLT = 16
};

/* Remote command */
#define GEN_ENTRY(name, hlp)                                                                                           \
    { #name, &cmd_handler::rcmd_##name, hlp }

class cmd_handler {
public:
    /* Table entry definition */
    class my_custom_command {
    public:
        /* command name */
        const char *name;
        /* command function */
        int (cmd_handler::*function)(int, char **, out_func, data_func);
        /* one line of help text */
        const char *help;
    };

    cmd_handler(iss::debugger::server_if &server, std::function<void(unsigned)> &stop_callback)
    : s(server)
    , t(s.get_target())
    , extended_protocol(false)
    , can_restart(false)
    , stop_callback(stop_callback) {}

    void attach();

    std::string search_memory(std::string const& in_buf);
    std::string threads(std::string const& in_buf);
    std::string read_registers(std::string const& in_buf);
    std::string write_registers(std::string const& in_buf);
    std::string read_single_register(std::string const& in_buf);
    std::string write_single_register(std::string const& in_buf);
    std::string read_memory(std::string const& in_buf);
    std::string write_memory(std::string const& in_buf);
    std::string running(std::string const& in_buf, bool blocking = true, bool vCont = false);
    int kill(std::string const& in_buf, std::string &out_buf);
    std::string thread_alive(std::string const& in_buf);
    void interrupt_target();
    int restart_target(std::string const& in_buf, std::string &out_buf);
    std::string detach(std::string const& in_buf);
    std::string query(std::string const& in_buf);
    std::string set(std::string const& in_buf);
    boost::optional<std::string> handle_extended(std::string const& in_buf);
    std::string breakpoint(std::string const& in_buf);
    int rcmd(const char *const in_buf, out_func of, data_func df);
    // TODO: change calls
    int rcmd_help(int argc, char *argv[], out_func of, data_func df);
    int rcmd_set(int argc, char *argv[], out_func of, data_func df);

    /* Encode return value */
    const char *to_string(int ret) {
        switch (ret) {
        case iss::Ok:
            return "OK";
        case iss::Err:
            return "E00";
        case iss::NotSupported:
            return "";
        default:
            assert(false);
            return nullptr;
        }
    }
    const char *to_string(iss::status ret) {
        switch (ret) {
        case iss::Ok:
            return "OK";
        case iss::Err:
            return "E00";
        case iss::NotSupported:
            return "";
        default:
            assert(false);
            return nullptr;
        }
    }

    iss::debugger::server_if &s;
    std::shared_ptr<target_adapter_if> t;
    encoder_decoder encdec;
    bool extended_protocol;
    bool can_restart;
    std::function<void(unsigned)> &stop_callback;
    std::unordered_map<uint64_t, unsigned> bp_map;
    std::array<const my_custom_command, 3> rp_remote_commands = {{
        /* Table of commands */
        GEN_ENTRY(help, "This help text"),
        GEN_ENTRY(set, "Set debug [level]"),
        {nullptr, nullptr, nullptr} // sentinel, end of table marker
    }};
};

} // namespace debugger
} // namspace iss

#endif /* _CMDHANDLER_H_ */
