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

#ifndef _GDB_SESSION_H_
#define _GDB_SESSION_H_

#include <boost/asio.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/bind/bind.hpp>
#include <chrono>
#include <cstdlib>
#include <iostream>
#include <iss/debugger/serialized_connection.h>
#include <thread>
#include <functional>

#include "cmdhandler.h"
#include "serialized_connection.h"
#include "server_if.h"
#include <iss/log_categories.h>

namespace iss {
namespace debugger {

using boost::asio::ip::tcp;

class gdb_session : public connection<std::string, std::string>::async_listener {
public:
    gdb_session(server_if *server_, boost::asio::io_context &io_service);

    virtual ~gdb_session() = default;

    tcp::socket &socket() { return conn_shptr->socket(); }

    int start();

    void receive_completed(const boost::system::error_code &e, std::string *data) override;

    void send_completed(const boost::system::error_code &e) override;

    bool message_completed(std::vector<char> &buffer) override;

protected:
    std::string check_packet(std::string &msg);

    void parse_n_execute(std::string &msg);

    void respond(std::string const& msg) {
        last_msg = msg;
        // std::this_thread::sleep_for(std::chrono::milliseconds(2));
        CLOG(TRACE, connection) << "Processed message, responding with '" << msg << "'";
        conn_shptr->write_data(msg);
        // conn_shptr->async_write(msg);
        conn_shptr->async_read();
    }

private:
    server_if *server;
    boost::shared_ptr<connection<std::string, std::string>> conn_shptr;
    std::string last_msg;
    cmd_handler handler;

    std::function<void(unsigned)> stop_callback;
};
}
}
#endif /* _GDB_SESSION_H_ */
