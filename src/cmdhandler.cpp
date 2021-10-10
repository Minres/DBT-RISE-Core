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

#include "iss/debugger/cmdhandler.h"
#include "iss/log_categories.h"
#include "util/ities.h"

#include <boost/tokenizer.hpp>
#include <cstdarg>
#include <numeric>
#include <sstream>
#include <stdexcept>

#ifdef _MSC_VER
// not #if defined(_WIN32) || defined(_WIN64) because we have strncasecmp in mingw
#define strncasecmp _strnicmp
#define strcasecmp _stricmp
#endif

using namespace iss::debugger;
using namespace boost;

void rp_console_output(const char *buf) {
    std::string msg(buf);
    if (buf[msg.size() - 1] == '\n') {
        CLOG(INFO, connection) << msg.substr(0, msg.size() - 1);
    } else {
        CLOG(INFO, connection) << buf;
    }
}

/* Funcions to stuff output value */

const unsigned CORE_ID = 0;

void cmd_handler::attach() {}

std::string cmd_handler::search_memory(std::string const& in_buf) {
    CLOG(TRACE, connection) << "executing " << __FUNCTION__;
    uint64_t addr;
    uint32_t pattern;
    uint32_t mask;
    const char *in;

    /* Format: taddr:PP,MM
     Search backwards starting at address addr for a match with the
     supplied pattern PP and mask MM. PP and MM are 4 bytes. addr
     must be at least 3 digits. */

    in = &in_buf[1];
    if (!encdec.dec_uint64(&in, &addr, ':')) {
        return std::string("E00");
    }
    if (!encdec.dec_uint32(&in, &pattern, ',')) {
        return std::string("E00");
    }
    if (!encdec.dec_uint32(&in, &mask, '\0')) {
        return std::string("E00");
    }
    return std::string("E01");
}

std::string cmd_handler::threads(std::string const& in_buf) {
    CLOG(TRACE, connection) << "executing " << __FUNCTION__;

    rp_thread_ref ref;
    const char *in;

    if (in_buf.size() == 1 || in_buf[2] == '-') {
        /* Either short or an obsolete form */
        return "";
    }

    /* Set thread */
    switch (in_buf[1]) {
    case 'c':
        in = in_buf.c_str() + 2;
        if (!encdec.dec_uint64(&in, &ref.val, '\0')) return "E00";
        return to_string(t->set_ctrl_thread(ref));
    case 'g':
        in = in_buf.c_str() + 2;
        if (!encdec.dec_uint64(&in, &ref.val, '\0')) return "E00";
        return to_string(t->set_gen_thread(ref));
    default:
        CLOG(ERR, connection) << __FUNCTION__ << ": Bad H command";
        return "";
    }
}

std::string cmd_handler::read_registers(std::string const& in_buf) {
    CLOG(TRACE, connection) << "executing " << __FUNCTION__;
    int ret;
    size_t len;
    std::vector<uint8_t> data_buf(MAX_DATABYTES);
    std::vector<uint8_t> avail_buf(MAX_DATABYTES);
    /* Get all registers. Format: 'g'. Note we do not do any data caching - all
     * caching is done by the debugger */
    ret = t->read_registers(data_buf, avail_buf);
    if (!ret) {
        return encdec.enc_regs(data_buf, avail_buf);
    } else
        return "E00";
}

std::string cmd_handler::write_registers(std::string const& in_buf) {
    CLOG(TRACE, connection) << "executing " << __FUNCTION__;
    size_t len;

    /* Write all registers. Format: 'GXXXXXXXXXX' */
    std::vector<uint8_t> data = encdec.dec_data(&in_buf[1]);
    if (data.empty())
        return "E00";
    else
        return to_string(t->write_registers(data));
}

std::string cmd_handler::read_single_register(std::string const& in_buf) {
    CLOG(TRACE, connection) << "executing " << __FUNCTION__;
    unsigned int reg_no;
    //    uint64_t avail=std::numeric_limits<uint64_t>::max();

    /* Get a single register. Format 'pNN' */
    int ret = encdec.dec_reg(&in_buf[1], &reg_no);
    if (!ret) return "E00";
    std::vector<uint8_t> data(8), avail(8);
    ret = t->read_single_register(reg_no, data, avail);
    switch (ret) {
    case iss::Ok:
        assert(data.size() <= MAX_DATABYTES);
        return encdec.enc_regs(data, avail);
    case iss::Err:
        return "E00";
    // handle targets non supporting single register read
    case iss::NotSupported:
        break;
    default:
        /* This should not happen */
        assert(false);
        break;
    }
    return "";
}

std::string cmd_handler::write_single_register(std::string const& in_buf) {
    CLOG(TRACE, connection) << "executing " << __FUNCTION__;
    unsigned int reg_no;
    /* Write a single register. Format: 'PNN=XXXXX' */
    std::vector<uint8_t> data = encdec.dec_reg_assignment(&in_buf[1], &reg_no);
    if (data.empty()) return "E00";
    assert(data.size() < MAX_DATABYTES);
    return to_string(t->write_single_register(reg_no, data));
}

std::string cmd_handler::read_memory(std::string const& in_buf) {
    CLOG(TRACE, connection) << "executing " << __FUNCTION__;
    int ret;
    size_t len;
    uint64_t addr;

    /* Read memory format: 'mAA..A,LL..LL' */
    if (!(ret = encdec.dec_mem(&in_buf[1], &addr, &len))) return "E00";

    /* Limit it so buggy gdbs will not complain */
    if (len > ((DBG_PRINTBUF_SIZE - 32) / 2)) len = (DBG_PRINTBUF_SIZE - 32) / 2;

    std::vector<uint8_t> data(len);
    ret = t->read_mem(addr, data);

    switch (ret) {
    case iss::Ok:
        assert(len <= MAX_DATABYTES);
        return encdec.enc_data(data);
    case iss::Err:
        return "E00";
    default:
        /* This should not happen */
        assert(false);
        break;
    }
    return "";
}

std::string cmd_handler::write_memory(std::string const& in_buf) {
    CLOG(TRACE, connection) << "executing " << __FUNCTION__;
    size_t cp;
    /* Write memory format: 'mAA..A,LL..LL:XX..XX' */
    if ((cp = in_buf.find_first_of(':', 1)) < in_buf.npos) return "E00";

    size_t len;
    uint64_t addr;
    int ret = encdec.dec_mem(&in_buf[1], &addr, &len);
    if (!ret || len > MAX_DATABYTES) return "E00";

    std::vector<uint8_t> data = encdec.dec_data(&in_buf[cp + 1]);
    if (!ret || len != data.size()) return "E00";
    return to_string(t->write_mem(addr, data));
}

std::string cmd_handler::running(std::string const& in_buf, bool blocking, bool vCont) {
    CLOG(TRACE, connection) << "executing " << __FUNCTION__ << "(" << in_buf << ")";
    auto step = false;
    uint32_t sig{TARGET_SIGNAL0};
    iss::status ret{iss::Err};
    /* 's' step from address
     * 'S' step from address with signal
     * 'c' continue from address
     * 'C' continue from address with signal
     */
    if (vCont) { // s:0;c
        auto cmds = util::split(in_buf, ';');
        for (auto &cmd : cmds) {
            rp_thread_ref thread{};
            if (cmd[0] == 't') {
                t->stop();
            } else {
                std::string c{cmd};
                if (cmd.find(':') != cmd.npos) { // the command is related to a specific thread
                    auto fields = util::split(cmd, ':');
                    // TODO: change to thread id instead of numbers
                    if (!encdec.dec_uint64(fields[1].c_str(), &thread.val)) return "E00";
                    c = fields[0];
                }
                if (c.size() > 1)
                    if (!encdec.dec_uint32(c.data() + 1, &sig)) return "E00";
                step |= (c[0] & 0x5f) == 'S'; // convert to uppercase and check
                ret = t->resume_from_current((c[0] & 0x5f) == 'S', sig, thread,
                                             blocking ? std::function<void(unsigned)>{} : stop_callback);
                if (ret == iss::Err) return to_string(ret);
            }
        }
    } else {                              // c[addr] or Csig[;addr]
        step = (in_buf[0] & 0x5f) == 'S'; // convert to uppercase and check
        const char *addr_ptr = nullptr;
        if (in_buf[0] < 'a') { // Uppercase, resume with signal, format Csig[;AA..AA], Ssig[;AA..AA], or Wsig[;AA..AA]
            const char *in = in_buf.data() + 1;
            if (strchr(in, ';')) {
                if (!encdec.dec_uint32(&in, &sig, ';')) return "E00";
                addr_ptr = in;
            } else {
                if (!encdec.dec_uint32(&in, &sig, '\0')) return "E00";
            }
        } else if (in_buf[1] != '\0') {
            addr_ptr = &in_buf[1];
        }
        if (addr_ptr) {
            uint64_t addr;
            if (!encdec.dec_uint64(&addr_ptr, &addr, '\0')) return "E00";
            ret = t->resume_from_addr(step, sig, addr, rp_thread_ref{},
                                      blocking ? std::function<void(unsigned)>{} : stop_callback);
        } else {
            ret = t->resume_from_current(step, sig, rp_thread_ref{},
                                         blocking ? std::function<void(unsigned)>{} : stop_callback);
        }
        if (ret != iss::Ok) return to_string(ret);
    }

    /* Now we have to wait for the target */
    /* Try a non-blocking wait first */
    bool running = false;
    ret = t->wait_non_blocking(running);
    if ((ret == iss::NotSupported || blocking) && !step) {
        // There is no partial wait facility for this target or there is no stop callback provided,
        // so use as blocking wait */
        ret = t->wait_blocking();
        running = false;
    }
    if (ret == iss::Err) return to_string(ret);
    if (!running) {
        /* We are done. The program has already stopped */
        return "S05"; // answer with SIGTRAP
    } else
        return "OK";
}

int cmd_handler::kill(std::string const& in_buf, std::string &out_buf) {
    CLOG(TRACE, connection) << "executing " << __FUNCTION__;
    int ret;

    s.shutdown();

    if (!extended_protocol) {
        //        dbg_sock_close();
        //
        //        if (!can_restart) {
        //            /* If the current target cannot restart, we have little choice
        //            but to exit right now. */
        //            CLOG(INFO, connection)<<__FUNCTION__<<": session killed.
        //            Exiting";
        //            dbg_sock_cleanup();
        //            exit(0);
        //        }

        CLOG(INFO, connection) << __FUNCTION__ << ": session killed. Will wait for a new connection";
        return 0;
    }

    CLOG(INFO, connection) << __FUNCTION__ << ": server restarting";

    /* Let us do our best while starting system */
    if (!can_restart) {
        /* Even if restart is not supported it is still worth calling connect */
        return -1;
    }

    ret = s.reset(CORE_ID);

    assert(ret != iss::NotSupported);

    if (ret != iss::Ok) {
        /* There is no point in continuing */
        CLOG(ERR, connection) << __FUNCTION__ << ": unable to restart target";
        out_buf = "E00";
        //        rp_putpkt(out_buf);
        //        dbg_sock_close();
        //
        //        if (!can_restart) {
        //            dbg_sock_cleanup();
        //            exit(1);
        //        }

        CLOG(INFO, connection) << __FUNCTION__ << ": will wait for a new connection";
        return 0;
    }
    return 1;
}

std::string cmd_handler::thread_alive(std::string const& in_buf) {
    CLOG(TRACE, connection) << "executing " << __FUNCTION__;
    int ret;
    bool alive;
    rp_thread_ref ref;
    const char *in;

    /* Is thread alive? */
    /* This is a deprecated feature of the remote debug protocol */
    in = &in_buf[1];
    if (!(ret = encdec.dec_uint64(&in, &ref.val, '\0'))) return "E00";

    ret = t->is_thread_alive(ref, alive);
    if (ret != iss::Ok) {
        return to_string(ret);
    } else {
        if (alive)
            return "OK";
        else
            return "E00";
    }
    return "E01";
}

int cmd_handler::restart_target(std::string const& in_buf, std::string &out_buf) {
    CLOG(TRACE, connection) << "executing " << __FUNCTION__;
    int ret;

    /* Restarting the target is only supported in the extended protocol. */
    if (!extended_protocol) return 0;

    assert(can_restart);

    /* Let us do our best to restart the system */
    if ((ret = s.reset(CORE_ID)) != iss::Ok) {
        /* There is no point to continuing */
        CLOG(ERR, connection) << __FUNCTION__ << ": unable to restart target";
        out_buf = "E00";
        CLOG(INFO, connection) << __FUNCTION__ << ": will wait for a new connection";
        return -1;
    }
    return 1;
}

std::string cmd_handler::detach(std::string const& in_buf) {
    CLOG(TRACE, connection) << "executing " << __FUNCTION__;
    int ret;

    s.shutdown();

    //    /* Note: The current GDB does not expect a reply */
    //    rp_putpkt(out_buf);
    //    dbg_sock_close();

    CLOG(INFO, connection) << __FUNCTION__ << ": debugger detaching";

    if (!can_restart) {
        /* If the current target cannot restart, we have little choice but
         to exit right now. */
        CLOG(INFO, connection) << __FUNCTION__ << ": target is not restartable. Exiting";
        exit(0);
    }

    CLOG(INFO, connection) << __FUNCTION__ << ": will wait for a new connection";
    return "";
}

std::string cmd_handler::query(std::string const& in_buf) {
    CLOG(TRACE, connection) << "executing " << __FUNCTION__;
    int ret;
    rp_thread_ref ref;
    rp_thread_info info;
    unsigned int len;
    uint32_t val;
    uint64_t addr;

    if (in_buf.size() == 1) {
        CLOG(ERR, connection) << __FUNCTION__ << ": bad 'q' command received";
        return "";
    }
    if (strncmp(in_buf.c_str() + 1, "Attached", 8) == 0) {
        return "1"; // always attached
    }
    if (strncmp(in_buf.c_str() + 1, "Offsets", 7) == 0) {
        uint64_t text;
        uint64_t data;
        uint64_t bss;

        /* Get the program segment offsets */
        ret = t->offsets_query(text, data, bss);
        if (ret == iss::Ok) {
            std::stringstream ss;
            ss << std::hex << "Text=" << text << ";Data=" << data << ";Bss=" << bss;
            return ss.str();
        } else {
            return to_string(ret);
        }
    }

    if (strncmp(in_buf.c_str() + 1, "CRC:", 4) == 0) {
        /* Find the CRC32 value of the specified memory area */
        const char *cp = &in_buf[5];
        if (!encdec.dec_uint64(&cp, &addr, ',')) {
            return "E00";
        }

        if (!encdec.dec_uint32(&cp, &len, '\0')) {
            return "E00";
        }
        ret = t->crc_query(addr, len, val);
        if (ret == iss::Ok) {
            std::stringstream ss;
            ss << "C" << std::hex << val;
            return ss.str();
        } else
            return to_string(ret);
    }

    if (strncmp(in_buf.c_str() + 1, "Symbol:", 7) == 0) {
        return "";
    }

    if (strncmp(in_buf.c_str() + 1, "TStatus:", 8) == 0) {
        return "T0;tnotrun:0";
    }

    if (strncmp(in_buf.c_str() + 1, "ThreadExtraInfo,", 16) == 0) {
        std::string data_buf;
        const char *in;

        if (!(ret = encdec.dec_uint64(&in, &ref.val, '\0'))) {
            return "E00";
        }

        ret = t->threadextrainfo_query(ref, data_buf);
        switch (ret) {
        case iss::Ok:
            return encdec.enc_data((unsigned char *)data_buf.c_str(), strlen(data_buf.c_str()));
            break;
        case iss::Err:
        case iss::NotSupported:
            return to_string(ret);
            break;
        default:
            assert(false);
            break;
        }
        return "";
    }

    if (strncmp(in_buf.c_str() + 1, "fThreadInfo", 11) == 0) {
        std::string buf;
        ret = t->threadinfo_query(1, buf);
        switch (ret) {
        case iss::Ok:
            return buf;
        case iss::NotSupported:
        case iss::Err:
            return to_string(ret);
        default:
            /* This should not happen */
            assert(false);
        }
        return "";
    }

    if (strncmp(in_buf.c_str() + 1, "sThreadInfo", 11) == 0) {
        std::string buf;
        ret = t->threadinfo_query(0, buf);
        switch (ret) {
        case iss::Ok:
            return buf;
        case iss::NotSupported:
        case iss::Err:
            return to_string(ret);
        default:
            /* This should not happen */
            assert(false);
        }
        return "";
    }

    if (strncmp(in_buf.c_str() + 1, "fProcessInfo", 12) == 0) {
        /* Get first string of process info */
        return "";
    }

    if (strncmp(in_buf.c_str() + 1, "sProcessInfo", 12) == 0) {
        /* Get subsequent string of process info */
        return "";
    }

    if (strncmp(in_buf.c_str() + 1, "Rcmd,", 5) == 0) {
        /* Remote command */
        std::stringstream ss;
        auto ret = rcmd(in_buf.c_str() + 6, rp_console_output, [&ss](std::string const& str) -> void { ss << str; });
        if (ret == iss::Ok && ss.str().size() > 0)
            return ss.str();
        else
            return to_string(ret);
    }
    if (strncmp(in_buf.c_str() + 1, "Xfer:features:read:", 19) == 0) {
        /* query features 'Xfer:features:read:annex:offset,length*/
        static std::string buf;
        const std::string start("l"); // last packet otherwise 'm'
        if (buf.size() == 0) t->target_xml_query(buf);
        auto col_pos = in_buf.find_first_of(':', 20);
        auto annex = in_buf.substr(20, col_pos - 20);
        auto cpos = in_buf.find_first_of(',', col_pos);
        auto offset_str = in_buf.substr(col_pos + 1, cpos - col_pos - 1);
        auto length_str = in_buf.substr(cpos + 1);
        // TODO: implement xml handling properly
        return start + buf;
    }
    if (strncmp(in_buf.c_str() + 1, "Supported", 9) == 0 && (in_buf[10] == ':' || in_buf[10] == '\0')) {
        std::string stdFeat("vContSupported+;hwbreak+;swbreak+;qXfer:features:read+");
        //        std::string stdFeat("vContSupported+;hwbreak+;swbreak+");
        // std::string stdFeat("hwbreak+;swbreak+");
        std::string buf;
        ret = t->packetsize_query(buf);
        switch (ret) {
        case iss::Ok:
            return stdFeat + ";" + buf;
        case iss::NotSupported:
        case iss::Err:
            return to_string(ret);
        default:
            /* This should not happen */
            assert(false);
        }
        return "";
    }

    switch (in_buf[1]) {
    case 'C':
        /* Current thread query */
        ret = t->current_thread_query(ref);
        if (ret == iss::Ok) {
            std::stringstream ss;
            ss << "QC" << std::hex << ref.val;
            return ss.str();
        } else
            return to_string(ret);
        break;
    case 'L':
        /* Thread list query */
        {
            rp_thread_ref arg;
            size_t max_found;
            size_t count;
            bool done;
            int first;
            ret = encdec.dec_list_query(&in_buf[2], &first, &max_found, &arg);
            if (!ret || max_found > 255) {
                return "E00";
            }
            std::vector<rp_thread_ref> threads(max_found);

            ret = t->thread_list_query(first, arg, threads, max_found, count, done);
            if (ret != iss::Ok || count > max_found) {
                return to_string(ret);
            }

            std::string res = encdec.enc_list_query_response(count, done, arg, threads);
            if (res.size())
                return res;
            else
                return "E00";
        }
        break;
    case 'P':
        /* Thread info query */
        {
            unsigned int mask;
            if (!(ret = encdec.dec_process_query(&in_buf[2], &mask, &ref))) return "E00";

            info.thread_id.val = 0;
            info.display[0] = 0;
            info.thread_name[0] = 0;
            info.more_display[0] = 0;

            if ((ret = t->process_query(mask, ref, info)) != iss::Ok) return to_string(ret);

            std::string ret = encdec.enc_process_query_response(mask, &ref, &info);
            if (ret.size())
                return ret;
            else
                return "E00";
            break;
        }
    default:
        /* Raw Query is a universal fallback */
        std::string buf;
        ret = t->raw_query(in_buf, buf);
        if (ret != iss::Ok) return to_string(ret);
        break;
    }
    return "";
}

std::string cmd_handler::set(std::string const& in_buf) {
    if (in_buf.find("QPassSignals:", 0) == 0) {
        /* Passing signals not supported */
        return ("");
    } else if (in_buf.find("QTDP", 0) == 0 || in_buf.find("QFrame", 0) == 0 || in_buf.find("QTStart", 0) == 0 ||
               in_buf.find("QTStop", 0) == 0 || in_buf.find("QTinit", 0) == 0 || in_buf.find("QTro", 0) == 0) {
        // a placeholder for tracepont packets, empty response to QTDP should
        // disable it */
        return ("");
    }
    return "";
}

std::string cmd_handler::breakpoint(std::string const& in_buf) {
    CLOG(TRACE, connection) << "executing " << __FUNCTION__;
    uint64_t addr;
    unsigned int len;
    int type;
    int ret;

    if (!(ret = encdec.dec_break(in_buf.c_str(), &type, &addr, &len))) return "E00";

    if (in_buf[0] == 'Z') {
        ret = t->add_break(type, addr, len);
        return "OK";
    } else {
        try {
            ret = t->remove_break(type, addr, len);
            return "OK";
        } catch (...) {
            return "E00";
        }
    }
}

void cmd_handler::interrupt_target() { t->stop(); }

/* Help function, generate help text from command table */
int cmd_handler::rcmd_help(int argc, char *argv[], out_func of, data_func df) {
    CLOG(TRACE, connection) << "executing " << __FUNCTION__;
    char buf[1000 + 1];
    char buf2[1000 + 1];
    int i = 0;

    encdec.enc_string("Remote command help:\n", buf, 1000);
    of("Remote command help:\n");
    df(buf);
    for (i = 0; rp_remote_commands[i].name; i++) {
#ifdef WIN32
        sprintf(buf2, "%-10s %s\n", rp_remote_commands[i].name, rp_remote_commands[i].help);
#else
        snprintf(buf2, 1000, "%-10s %s\n", rp_remote_commands[i].name, rp_remote_commands[i].help);
#endif
        encdec.enc_string(buf2, buf, 1000);
        of(buf2);
        df(buf);
    }
    std::vector<target_adapter_if::custom_command> cc = t->custom_commands();
    for (i = 0; i < cc.size(); i++) {
#ifdef WIN32
        sprintf(buf2, "%-10s %s\n", cc[i].name, cc[i].help);
#else
        snprintf(buf2, 1000, "%-10s %s\n", cc[i].name, cc[i].help);
#endif
        encdec.enc_string(buf2, buf, 1000);
        of(buf2);
        df(buf);
    }
    return iss::Ok;
}

/* Set function, set debug level */
int cmd_handler::rcmd_set(int argc, char *argv[], out_func of, data_func df) {
    CLOG(TRACE, connection) << "executing " << __FUNCTION__;
    char buf[1000 + 1];
    char buf2[1000 + 1];

    if (argc == 1) {
        sprintf(buf2, "Missing argument to set command.\n");
        encdec.enc_string(buf2, buf, 1000);
        of(buf2);
        df(buf);
        return iss::Err;
    }

    if (strcmp("debug", argv[1]) != 0) {
        sprintf(buf2, "Undefined set command: \"%s\"\n", argv[1]);
        encdec.enc_string(buf2, buf, 1000);
        of(buf2);
        df(buf);
        return iss::Err;
    }

    if (argc != 3) {
        sprintf(buf2, "Wrong arguments for debug command.\n");
        encdec.enc_string(buf2, buf, 1000);
        of(buf2);
        df(buf);
        return iss::Err;
    }

    if (strcmp("0", argv[2]) == 0 || strcasecmp("NONE", argv[2]) == 0)
        LOGGER(DEFAULT)::reporting_level() = logging::NONE;
    else if (strcmp("1", argv[2]) == 0 || strcasecmp("FATAL", argv[2]) == 0)
        LOGGER(DEFAULT)::reporting_level() = logging::FATAL;
    else if (strcmp("2", argv[2]) == 0 || strcasecmp("ERROR", argv[2]) == 0)
        LOGGER(DEFAULT)::reporting_level() = logging::ERR;
    else if (strcmp("3", argv[2]) == 0 || strcasecmp("WARNING", argv[2]) == 0)
        LOGGER(DEFAULT)::reporting_level() = logging::WARN;
    else if (strcmp("4", argv[2]) == 0 || strcasecmp("INFO", argv[2]) == 0)
        LOGGER(DEFAULT)::reporting_level() = logging::INFO;
    else if (strcmp("5", argv[2]) == 0 || strcasecmp("DEBUG", argv[2]) == 0)
        LOGGER(DEFAULT)::reporting_level() = logging::DEBUG;
    else if (strcmp("6", argv[2]) == 0 || strcasecmp("TRACE", argv[2]) == 0)
        LOGGER(DEFAULT)::reporting_level() = logging::TRACE;
    else {
        sprintf(buf2, "Invalid debug level: \"%s\"\n", argv[2]);
        encdec.enc_string(buf2, buf, 1000);
        of(buf2);
        df(buf);
        return iss::Err;
    }
    // print unconditional
    LOGGER(DEFAULT)().get(logging::INFO) << "Log level set to " << LOGGER(DEFAULT)::reporting_level();
    return iss::Ok;
}

/* Target method */
int cmd_handler::rcmd(const char *const in_buf, out_func of, data_func df) {
    CLOG(TRACE, connection) << "executing " << __FUNCTION__;
    int count = 0;
    int i;
    char *args[MAXARGS];
    char *ptr;
    unsigned int ch;
    char buf[1000 + 1];
    char *s;

    CLOG(TRACE, connection) << "command '" << in_buf << "'";

    if (strlen(in_buf)) {
        /* There is something to process */
        /* TODO: Handle target specific commands, such as flash erase, JTAG
     control, etc. */
        /* A single example "flash erase" command is partially implemented
     here as an example. */

        /* Turn the hex into ASCII */
        ptr = const_cast<char *>(in_buf);
        s = buf;
        while (*ptr) {
            if (encdec.dec_byte(ptr, &ch) == 0) return iss::Err;
            *s++ = ch;
            ptr += 2;
        }
        *s = '\0';
        CLOG(DEBUG, connection) << "command '" << buf << "'";

        /* Split string into separate arguments */
        ptr = buf;
        args[count++] = ptr;
        while (*ptr) {
            /* Search to the end of the string */
            if (*ptr == ' ') {
                /* Space is the delimiter */
                *ptr = 0;
                if (count >= MAXARGS) return iss::Err;
                args[count++] = ptr + 1;
            }
            ptr++;
        }
        /* Search the command table, and execute the function if found */
        CLOG(TRACE, connection) << "executing target dependant command '" << args[0] << "'";

        /* Search the target command table first, such that we allow target to
         * override the general command.  */
        for (i = 0; i < t->custom_commands().size(); i++) {
            if (strcmp(args[0], t->custom_commands()[i].name) == 0)
                return t->custom_commands()[i].function(count, args, of, df);
        }

        for (i = 0; rp_remote_commands[i].name; i++) {
            if (strcmp(args[0], rp_remote_commands[i].name) == 0) {
                int (cmd_handler::*fptr)(int, char **, out_func, data_func) = rp_remote_commands[i].function;
                return (this->*fptr)(count, args, of, df);
            }
        }
        return iss::NotSupported;
    }
    return iss::Err;
}

boost::optional<std::string> cmd_handler::handle_extended(std::string const& in_buf) {
    if (in_buf.find("vCtrlC", 0) == 0) {
        t->stop();
        return boost::optional<std::string>{"OK"};
        // to be implemented later
    } else if (in_buf.find("vCont", 0) == 0) {
        if (in_buf[5] == '?') {
            // return boost::optional<std::string>{"vCont;c;C;t;s;S"};
            return boost::optional<std::string>{""};
        } else if (in_buf.size() == 5) {
            std::string ret = running("c", false, true);
            if (ret.size() > 0) return ret;
            return boost::optional<std::string>{};
        } else {
            std::string ret = running(in_buf.substr(6), false, true);
            if (ret.size() > 0) return ret;
            return boost::optional<std::string>{};
        }
    } else if (in_buf.find("vRun", 0) == 0) {
        t->restart();
        return boost::optional<std::string>{"S05"};
    } else if (in_buf.find("vAttach", 0) == 0) {
        return boost::optional<std::string>{"S05"};
    } else if (in_buf.find("vCtrlC", 0) == 0) {
        t->stop();
        return boost::optional<std::string>{"OK"};
    } else
        return boost::optional<std::string>{""};
    // not suppported:
    // vFile
    // vFlashErase
    // vFlashWrite
    // vFlashDone
}
