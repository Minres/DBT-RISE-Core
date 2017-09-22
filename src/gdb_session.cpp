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

#include <algorithm>
#include <exception>
#include <iomanip>
#include <iostream>
#include <iss/debugger/gdb_session.h>
#include <string>
#include <util/logging.h>

namespace iss {
namespace debugger {
class packet_parse_error : public std::exception {};
}
}
using namespace iss::debugger;
using namespace std;

const int THREAD_ID = 1;
const int CORE_ID = 0;
const std::string ack("+");
const std::string sig_trap("S05");

template <class TContainer> bool begins_with(const TContainer &input, const TContainer &match) {
    return input.size() >= match.size() && equal(match.begin(), match.end(), input.begin());
}

inline bool compare(string input, string match, size_t offset = 0) {
    return (input.length() >= (offset + match.length())) && (input.compare(offset, match.length(), match) == 0);
}

inline std::vector<std::string> split(const std::string &s, char seperator) {
    std::vector<std::string> output;
    std::string::size_type prev_pos = 0, pos = 0;
    while ((pos = s.find(seperator, pos)) != std::string::npos) {
        std::string substring(s.substr(prev_pos, pos - prev_pos));
        output.push_back(substring);
        prev_pos = ++pos;
    }
    output.push_back(s.substr(prev_pos, pos - prev_pos)); // Last word
    return output;
}

struct gdb_resp_msg {
    gdb_resp_msg() : check_sum(0), body(false) {}
    void add(uint8_t m) {
        if (m == '#' || m == '$' || m == '}') {
            buffer.push_back('}');
            m ^= 0x20;
        }
        buffer.push_back(m);
        check_sum += m;
    }
    char get_checksum_char(int idx) const {
        assert(idx == 0 || idx == 1);
        const char c = (check_sum >> (4 - 4 * idx)) & 0x0F;
        assert(c < 16);
        return c > 9 ? (c + 'a' - 10) : (c + '0');
    }
    gdb_resp_msg &operator<<(const char *msg) {
        body = true;
        while (msg && *msg) add(*(msg++));
        return *this;
    }
    template <typename T> gdb_resp_msg &operator<<(T val) {
        stringstream ss;
        ss << std::hex << val;
        this->operator<<(ss.str().c_str());
        return *this;
    }
    unsigned char operator[](unsigned int idx) const { return buffer.at(idx); }
    operator bool() { return /*ack||*/ body; }
    operator std::string() {
        std::stringstream ss;
        if (body) {
            ss << "$";
            for (std::vector<uint8_t>::const_iterator it = buffer.begin(); it != buffer.end(); ++it) ss << *it;
            ss << "#" << get_checksum_char(0) << get_checksum_char(1);
        }
        return ss.str();
    }

protected:
    std::vector<uint8_t> buffer;
    uint8_t check_sum;
    bool body;
};

int gdb_session::start() {
    conn_shptr->add_listener(shared_from_this());
    boost::asio::ip::tcp::endpoint endpoint = conn_shptr->socket().remote_endpoint();
    CLOG(TRACE, connection) << "gdb_session::start(), got connected to remote port " << endpoint.port() << " on "
                            << endpoint.address().to_string();
    conn_shptr->async_read(); // start listening
    return 0;
}

void gdb_session::send_completed(const boost::system::error_code &e) {
    if (!e) {
        conn_shptr->async_read();
    } else {
        LOG(ERROR) << e.message() << "(" << e << ")";
    }
}

bool is_all_hex(char *input) { // destroys input
    return (strtok(input, "0123456789ABCDEFabcdef") == NULL);
}

bool gdb_session::message_completed(std::vector<char> &buffer) {
    //	std::cout << "Checking for "; for (auto i = buffer.begin(); i !=
    // buffer.end(); ++i) std::cout << (int)*i << '('<<*i<<")
    //";std::cout<<std::endl;
    size_t s = buffer.size();
    if (s == 1 && (buffer[0] == '+' || buffer[0] == '-')) return true;
    if (s == 2 && (buffer[0] == -1 && buffer[1] == -13)) return true;
    if (s > 4 && buffer[0] == '$' && buffer[s - 3] == '#') {
        return true;
    }
    return false;
}

void gdb_session::receive_completed(const boost::system::error_code &e, std::string *msg) {
    if (e.value() == 2) {
        CLOG(WARNING, connection) << "Client closed connection (" << e.message() << ")";
        // TODO: cleanup settings like: server.remove_breakpoint(CORE_ID, 0);
        handler.t->close();
        return;
    } else if (e) {
        CLOG(ERROR, connection) << "Communication error (" << e.message() << ")";
        handler.t->close();
        return;
    }
    if (msg->compare("+") == 0) {
        CLOG(TRACE, connection) << "Received ACK";
        last_msg = "";
        conn_shptr->async_read();
    } else if (msg->compare("-") == 0) {
        CLOG(TRACE, connection) << "Received NACK, repeating msg '" << last_msg << "'";
        conn_shptr->write_data(&last_msg);
        conn_shptr->async_read();
    } else if (msg->at(0) == -1 && msg->at(1) == -13) {
        CLOG(TRACE, connection) << "Received BREAK, interrupting target";
        handler.t->stop();
        respond("S05");
    } else {
        CLOG(TRACE, connection) << "Received packet '" << *msg << "', processing it";
        std::string data = check_packet(*msg);
        if (data.size()) {
            conn_shptr->write_data(&ack);
            parse_n_execute(data);
        } else
            respond("-");
    }
    return;
}

void gdb_session::parse_n_execute(std::string &data) {
    try {
        gdb_resp_msg resp;
        std::string out_buf;
        int do_connect;
        bool do_reinitialize = false, input_error = false;
        switch (data[0]) {
        case '!':
            /* Set extended operation */
            CLOG(DEBUG, connection) << __FUNCTION__ << ": switching to extended protocol mode";
            if (handler.can_restart) {
                handler.extended_protocol = true;
                resp << "OK";
            } else {
                /* Some GDBs will accept any response as a good one. Let us bark in the
                 * log at least */
                CLOG(ERROR, connection) << __FUNCTION__ << ": extended operations required, but not supported";
            }
            break;
        case '?':
            /* Report the last signal status */
            resp << "T0" << ABORTED;
            break;
        case 'A':
            /* Set the argv[] array of the target */
            // TODO: implement this!
            break;
        case 'C':
        case 'S':
        case 'c':
        case 's':
            resp << handler.running(data);
            break;
        case 'D':
            resp << handler.detach(data); // TODO: pass socket handling
            do_reinitialize = true;
            break;
        case 'g':
            resp << handler.read_registers(data);
            break;
        case 'G':
            resp << handler.write_registers(data);
            break;
        case 'H':
            resp << handler.threads(data);
            break;
        case 'k':
            do_connect = handler.kill(data, out_buf); // TODO: pass socket handling
            if (do_connect == -1) input_error = true;
            if (!do_connect) do_reinitialize = true;
            resp << out_buf;
            break;
        case 'm':
            resp << handler.read_memory(data);
            break;
        case 'M':
            resp << handler.write_memory(data);
            break;
        case 'p':
            resp << handler.read_single_register(data);
            break;
        case 'P':
            resp << handler.write_single_register(data);
            break;
        case 'q':
            resp << handler.query(data);
            break;
        case 'Q':
            resp << handler.set(data);
            break;
        case 'R':
            do_connect = handler.restart_target(data, out_buf); // TODO: pass socket handling
            if (do_connect == -1) do_reinitialize = true;
            resp << out_buf;
            if (do_connect) break;
            break;
        case 't':
            resp << handler.search_memory(data);
            break;
        case 'T':
            resp << handler.thread_alive(data);
            break;
        case 'v':
            resp << handler.handle_extended(data);
            break;
        case 'Z':
        case 'z':
            resp << handler.breakpoint(data);
            break;
        default:
            resp << "";
            break;
        }
        respond(resp);
    } catch (boost::system::system_error const &e1) {
        CLOG(ERROR, connection) << "Caught boost error " << e1.what();
    } catch (std::exception const &e2) {
        CLOG(ERROR, connection) << "Caught std::exception (" << typeid(e2).name() << "): " << e2.what();
    }
}

std::string gdb_session::check_packet(std::string &msg) {
    std::string::size_type start = msg.find_first_of('$');
    std::string::size_type end = msg.find_first_of('#', start);
    unsigned checksum = 0;
    for (unsigned i = start + 1; i < end; i++) checksum += msg.at(i);
    std::istringstream in(msg.substr(end + 1, 2));
    unsigned long xmit;
    in >> std::hex >> xmit;
    if (xmit == (checksum & 0xff)) return msg.substr(start + 1, end - 1);
    // throw(iss::debugger::packet_parse_error());
    return "";
}
