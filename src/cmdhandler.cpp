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

#include <iss/debugger/cmdhandler.h>

#include <iss/log_categories.h>
#include <numeric>
#include <stdarg.h>
#include <stdexcept>

using namespace iss::debugger;

void rp_console_output(const char *buf) { CLOG(INFO, connection) << buf; }

void rp_data_output(const char *buf) {}
/* Funcions to stuff output value */

const unsigned CORE_ID = 0;

void cmd_handler::attach() {}

std::string cmd_handler::search_memory(const std::string in_buf) {
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

std::string cmd_handler::threads(const std::string in_buf) {
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
        CLOG(ERROR, connection) << __FUNCTION__ << ": Bad H command";
        return "";
    }
}

std::string cmd_handler::read_registers(const std::string in_buf) {
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

std::string cmd_handler::write_registers(const std::string in_buf) {
    CLOG(TRACE, connection) << "executing " << __FUNCTION__;
    size_t len;

    /* Write all registers. Format: 'GXXXXXXXXXX' */
    std::vector<uint8_t> data = encdec.dec_data(&in_buf[1]);
    if (data.empty())
        return "E00";
    else
        return to_string(t->write_registers(data));
}

std::string cmd_handler::read_single_register(const std::string in_buf) {
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
        assert(0);
        break;
    }
    return "";
}

std::string cmd_handler::write_single_register(const std::string in_buf) {
    CLOG(TRACE, connection) << "executing " << __FUNCTION__;
    unsigned int reg_no;
    /* Write a single register. Format: 'PNN=XXXXX' */
    std::vector<uint8_t> data = encdec.dec_reg_assignment(&in_buf[1], &reg_no);
    if (data.empty()) return "E00";
    assert(data.size() < MAX_DATABYTES);
    return to_string(t->write_single_register(reg_no, data));
}

std::string cmd_handler::read_memory(const std::string in_buf) {
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
        assert(0);
        break;
    }
    return "";
}

std::string cmd_handler::write_memory(const std::string in_buf) {
    CLOG(TRACE, connection) << "executing " << __FUNCTION__;
    size_t cp;
    /* Write memory format: 'mAA..A,LL..LL:XX..XX' */
    if ((cp = in_buf.find_first_of(':', 1)) < in_buf.npos) return "E00";

    size_t len;
    uint64_t addr;
    int ret = encdec.dec_mem(&in_buf[1], &addr, &len);
    if (!ret || len > MAX_DATABYTES) return "E00";

    size_t len1;
    std::vector<uint8_t> data = encdec.dec_data(&in_buf[cp + 1]);
    if (!ret || len != len1) return "E00";
    return to_string(t->write_mem(addr, data));
}

std::string cmd_handler::running(const std::string in_buf) {
    CLOG(TRACE, connection) << "executing " << __FUNCTION__;
    bool step;
    std::string status_string;
    uint32_t sig;
    bool running;
    bool implemented;
    const char *addr_ptr;
    uint64_t addr;
    int ret;
    const char *in;

    /* 's' step from address
     * 'S' step from address with signal
     * 'c' continue from address
     * 'C' continue from address with signal
     */

    step = (in_buf[0] == 'S' || in_buf[0] == 's');

    addr_ptr = nullptr;

    if (in_buf[0] == 'C' || in_buf[0] == 'S' || in_buf[0] == 'W') {
        /*
         * Resume with signal.
         * Format Csig[;AA..AA], Ssig[;AA..AA], or Wsig[;AA..AA]
         */

        in = &in_buf[1];
        if (strchr(in, ';')) {
            if (!encdec.dec_uint32(&in, &sig, ';')) return "E00";
            addr_ptr = in;
        } else {
            if (!encdec.dec_uint32(&in, &sig, '\0')) return "E00";
        }
    } else {
        sig = TARGET_SIGNAL0;
        if (in_buf[1] != '\0') addr_ptr = &in_buf[1];
    }

    if (addr_ptr) {
        if (!encdec.dec_uint64(&addr_ptr, &addr, '\0')) return "E00";
        ret = t->resume_from_addr(step, sig, addr);
    } else {
        ret = t->resume_from_current(step, sig);
    }

    if (ret == iss::Ok)        // answer with SIGTRAP
        status_string = "T05"; // step?"T05":"T06";

    if (ret != iss::Ok) {
        return to_string(ret);
    }

    /* Now we have to wait for the target */
    /* Try a non-blocking wait first */
    ret = t->wait_non_blocking(status_string, rp_console_output, running);
    if (ret == iss::Err) return to_string(ret);
    ret = iss::NotSupported; // TODO: Fix, needs to send back stop if finished
    if (ret == iss::NotSupported && !step) {
        /* There is no partial wait facility for this target, so use as blocking
         * wait */
        ret = t->wait_blocking(status_string, rp_console_output);
        assert(ret != iss::NotSupported);
        return ret == iss::Ok ? status_string : to_string(ret);
    }
    if (!running) {
        /* We are done. The program has already stopped */
        return status_string;
    }
    return "";
}

int cmd_handler::kill(const std::string in_buf, std::string &out_buf) {
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

    CLOG(INFO, connection) << __FUNCTION__ << ": remote proxy restarting";

    /* Let us do our best while starting system */
    if (!can_restart) {
        /* Even if restart is not supported it is still worth calling connect */
        return -1;
    }

    ret = s.reset(CORE_ID);

    assert(ret != iss::NotSupported);

    if (ret != iss::Ok) {
        /* There is no point in continuing */
        CLOG(ERROR, connection) << __FUNCTION__ << ": unable to restart target";
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

std::string cmd_handler::thread_alive(const std::string in_buf) {
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

int cmd_handler::restart_target(const std::string in_buf, std::string &out_buf) {
    CLOG(TRACE, connection) << "executing " << __FUNCTION__;
    int ret;

    /* Restarting the target is only supported in the extended protocol. */
    if (!extended_protocol) return 0;

    assert(can_restart);

    /* Let us do our best to restart the system */
    if ((ret = s.reset(CORE_ID)) != iss::Ok) {
        /* There is no point to continuing */
        CLOG(ERROR, connection) << __FUNCTION__ << ": unable to restart target";
        out_buf = "E00";
        CLOG(INFO, connection) << __FUNCTION__ << ": will wait for a new connection";
        return -1;
    }
    return 1;
}

std::string cmd_handler::detach(const std::string in_buf) {
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

std::string cmd_handler::query(const std::string in_buf) {
    CLOG(TRACE, connection) << "executing " << __FUNCTION__;
    int ret;
    rp_thread_ref ref;
    rp_thread_info info;
    unsigned int len;
    uint32_t val;
    uint64_t addr;

    if (in_buf.size() == 1) {
        CLOG(ERROR, connection) << __FUNCTION__ << ": bad 'q' command received";
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
        return "";
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
            assert(0);
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
            assert(0);
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
            assert(0);
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
        return to_string(rcmd(in_buf.c_str() + 6, rp_console_output, rp_data_output));
    }

    if (strncmp(in_buf.c_str() + 1, "Supported", 9) == 0 && (in_buf[10] == ':' || in_buf[10] == '\0')) {
        //    	std::string stdFeat("vContSupported+;hwbreak+;swbreak+");
        std::string stdFeat("hwbreak+;swbreak+");
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
            assert(0);
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

std::string cmd_handler::set(const std::string in_buf) {
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

std::string cmd_handler::breakpoint(const std::string in_buf) {
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
    of(buf);
    for (i = 0; rp_remote_commands[i].name; i++) {
#ifdef WIN32
        sprintf(buf2, "%-10s %s\n", rp_remote_commands[i].name, rp_remote_commands[i].help);
#else
        snprintf(buf2, 1000, "%-10s %s\n", rp_remote_commands[i].name, rp_remote_commands[i].help);
#endif
        encdec.enc_string(buf2, buf, 1000);
        of(buf);
    }
    std::vector<target_adapter_if::custom_command> cc = t->custom_commands();
    for (i = 0; i < cc.size(); i++) {
#ifdef WIN32
        sprintf(buf2, "%-10s %s\n", t->custom_commands[i].name, t->custom_commands[i].help);
#else
        snprintf(buf2, 1000, "%-10s %s\n", cc[i].name, cc[i].help);
#endif
        encdec.enc_string(buf2, buf, 1000);
        of(buf);
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
        of(buf);
        return iss::Ok;
    }

    if (strcmp("debug", argv[1]) != 0) {
        sprintf(buf2, "Undefined set command: \"%s\"\n", argv[1]);
        encdec.enc_string(buf2, buf, 1000);
        of(buf);
        return iss::Ok;
    }

    if (argc != 3) {
        sprintf(buf2, "Wrong arguments for debug command.\n");
        encdec.enc_string(buf2, buf, 1000);
        of(buf);
        return iss::Ok;
    }

    if (strcmp("0", argv[2]) == 0)
        LOGGER(DEFAULT)::reporting_level() = logging::NONE;
    else if (strcmp("1", argv[2]) == 0)
        LOGGER(DEFAULT)::reporting_level() = logging::FATAL;
    else if (strcmp("2", argv[2]) == 0)
        LOGGER(DEFAULT)::reporting_level() = logging::ERROR;
    else if (strcmp("3", argv[2]) == 0)
        LOGGER(DEFAULT)::reporting_level() = logging::WARNING;
    else if (strcmp("4", argv[2]) == 0)
        LOGGER(DEFAULT)::reporting_level() = logging::INFO;
    else if (strcmp("5", argv[2]) == 0)
        LOGGER(DEFAULT)::reporting_level() = logging::DEBUG;
    else if (strcmp("6", argv[2]) == 0)
        LOGGER(DEFAULT)::reporting_level() = logging::TRACE;
    else {
        sprintf(buf2, "Invalid debug level: \"%s\"\n", argv[2]);
        encdec.enc_string(buf2, buf, 1000);
        of(buf);
        return iss::Ok;
    }

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

    CLOG(DEBUG, connection) << __FUNCTION__ << ": handle_rcmd()";
    CLOG(DEBUG, connection) << "command '" << in_buf << "'";

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
            if (encdec.dec_byte(in_buf, &ch) == 0) return iss::Err;
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
        CLOG(DEBUG, connection) << "executing target dependant command '" << args[0] << "'";

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

std::string cmd_handler::handle_extended(const std::string in_buf) {
    if (in_buf.find("vCtrlC", 0) == 0) {
        t->stop();
        return "OK";
        // to be implemented later
        //	} else if(in_buf.find("vCont", 0)==0){
        //		if(in_buf[5]=='?') return "vCont;cst";
    } else if (in_buf.find("vRun", 0) == 0) {
        t->restart();
        return "S05";
    } else if (in_buf.find("vAttach", 0) == 0) {
        return "S05";
    } else
        return "";
    // not suppported:
    // vFile
    // vFlashErase
    // vFlashWrite
    // vFlashDone
}
