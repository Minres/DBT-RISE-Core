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

#ifndef _SERVER_H_
#define _SERVER_H_

#include "server_base.h"
#include <boost/asio.hpp>
#include <boost/asio/io_context.hpp>
#include <boost/thread.hpp>
#include <ctime>
#include <util/logging.h>

namespace iss {
namespace debugger {

template <class SESSION> class server : public server_base {
public:
    static void run_server(iss::debugger_if *vm, unsigned int port) {
        if (get() != NULL) {
            LOG(FATAL) << "server already initialized";
        }
        LOG(DEBUG) << "starting server listening on port "<<port;
        get(new server<SESSION>(vm, port));
    }

    static server<SESSION> *get() { return get(NULL); }

    unsigned short get_port_nr() { return acceptor.local_endpoint().port(); }

    void shutdown() override {
        delete work_ctrl;
        work_ctrl = nullptr;
        // Wait for all threads in the pool to exit.
        threads.join_all();
        // allow the synchronizer to run
        boost::this_thread::sleep(boost::posix_time::milliseconds(10));
    }

protected:
    static server<SESSION> *get(server<SESSION> *srv) {
        static server<SESSION> *s = NULL;
        if (srv != NULL && s == NULL) s = srv;
        return s;
    }

    server(iss::debugger_if *vm, unsigned short port)
    : server_base(vm)
    , acceptor(io_service, boost::asio::ip::tcp::endpoint(boost::asio::ip::tcp::v4(), port)) {
        // Create a pool of threads to run all of the io_services.
        std::size_t thread_pool_size_ = 2;
        work_ctrl = new boost::asio::io_context::work(io_service);
        boost::thread::hardware_concurrency();
        for (std::size_t i = 0; i < thread_pool_size_; ++i) {
            threads.create_thread([this]() -> void { io_service.run(); });
        }
        createNewSession();
    };

    void createNewSession() {
        boost::shared_ptr<SESSION> new_session(new SESSION(this, io_service));
        acceptor.async_accept(new_session->socket(),
                boost::bind(&server<SESSION>::handle_accept, this, new_session,
                  boost::asio::placeholders::error));
    }

    void shutDown() {
        // request shutdown of simulation
        server_base::shutdown();
        // stop the io acceptor
        io_service.stop();
    }

private:
    /// Handle completion of a accept operation by starting a session.
    void handle_accept(boost::shared_ptr<SESSION> session, const boost::system::error_code &e) {
        if (!e) {
            // Start an accept operation for a new connection. If it finishes create a
            // new session
            if (!session->start()) createNewSession();
        } else {
            // An error occurred. Log it and return. Since we are not starting a new
            // accept operation the io_service will run out of work to do and the
            // thread will exit.
            LOG(ERR) << e.message();
        }
    }
    // server related members
    boost::asio::io_context io_service;
    boost::asio::io_service::work *work_ctrl;
    boost::asio::ip::tcp::acceptor acceptor;
    boost::thread_group threads;
};

} // namespace debugger
} // namspace iss

#endif /* _SERVER_H_ */
