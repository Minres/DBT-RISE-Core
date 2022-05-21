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

#ifndef _TARGET_ADAPTER_IF_H_
#define _TARGET_ADAPTER_IF_H_

#include <array>
#include <functional>
#include <iss/vm_types.h>
#include <string>
#include <vector>

namespace iss {
namespace debugger {

/* Function to do console output from wait methods */
// using out_func = void (*)(const char *string);
using out_func = std::function<void(const char *string)>;
/* Function to transfer data received as qRcmd response */
// using data_func = void (*)(const char *string);
using data_func = std::function<void(const std::string &)>;

/* Function to do logging */
using log_func = void (*)(int level, const char *string, ...);

class rp_thread_ref {
public:
    uint64_t val = std::numeric_limits<uint64_t>::max();
};

class rp_thread_info {
public:
    rp_thread_ref thread_id;
    int exists;
    std::string display;
    std::string thread_name;
    std::string more_display;
};

class target_adapter_if {
public:
    /* Table entry definition */
    class custom_command {
    public:
        /* command name */
        const char *name;
        /* command function */
        // int (*function)(int, char **, out_func, data_func);
        std::function<int(int, char **, out_func, data_func)> function;
        /* one line of help text */
        const char *help;
    };

    virtual ~target_adapter_if() = default;

    /* return table of remote commands */
    virtual const std::vector<custom_command> &custom_commands() = 0;

    virtual void add_custom_command(custom_command &&cmd) = 0;
    /*======================   Help/Debug  =======================*/

    /* Help, argument is a pointer to itself */
    virtual void help(const char *prog_name) = 0;

    /*=========   Open/Close/Connect/Disconnect  ==============*/

    /* Start target stub and provide run time parameters
     in time tested manner, does not assume actually
     connecting to target.  */
    virtual iss::status open(int argc, char *const agrv[], const char *prog_name, log_func log_fn) = 0;

    /* Close target stub: if target is still connected disconnect and
     leave it running */
    virtual void close() = 0;

    /* Actually connect to a target and return status string; */
    virtual iss::status connect(bool &can_restart) = 0;

    /* Disconnect from a target a leave it running */
    virtual iss::status disconnect() = 0;

    /*=================== Start/Stop =========================*/

    /* Kill target: disconnect from a target and leave it waiting
     for a command. Target output is ignored.

     Restart: start target all over again.

     Stop: break into running target

     Note these commands are used in following sequences only

     1. kill, close, terminate proxy
     2. kill, restart, connect
     3. restart, connect
     4. stop, wait */

    /* Kill target: disconnect from a target and leave it waiting
     for a command. It is expected that either close or wait or
     connect will follow after kill to get last status_string */
    virtual void kill() = 0;

    /* Restart target and return status string */
    virtual iss::status restart() = 0;

    /* Stop target. E.g. send ^C or BREAK to target - note
     it has to be followed either by wait or connect in order to
     to get last status_string */
    virtual void stop() = 0;

    /*============== Thread Control ===============================*/

    /* Set generic thread */
    virtual iss::status set_gen_thread(rp_thread_ref &thread) = 0;

    /* Set control thread */
    virtual iss::status set_ctrl_thread(rp_thread_ref &thread) = 0;

    /* Get thread status */
    virtual iss::status is_thread_alive(rp_thread_ref &thread, bool &alive) = 0;

    /*============= Register Access ================================*/

    /* Read all registers. buf is 4-byte aligned and it is in
     target byte order. If  register is not available
     corresponding bytes in avail_buf are 0, otherwise
     avail buf is 1 */
    virtual iss::status read_registers(std::vector<uint8_t> &data_buf, std::vector<uint8_t> &avail_buf) = 0;

    /* Write all registers. buf is 4-byte aligned and it is in target
     byte order */
    virtual iss::status write_registers(const std::vector<uint8_t> &buf) = 0;

    /* Read one register. buf is 4-byte aligned and it is in
     target byte order. If  register is not available
     corresponding bytes in avail_buf are 0, otherwise
     avail buf is 1 */
    virtual iss::status read_single_register(unsigned int reg_no, std::vector<uint8_t> &buf,
                                             std::vector<uint8_t> &avail_buf) = 0;

    /* Write one register. buf is 4-byte aligned and it is in target byte
     order */
    virtual iss::status write_single_register(unsigned int reg_no, const std::vector<uint8_t> &buf) = 0;

    /*=================== Memory Access =====================*/

    /* Read memory, buf is 4-bytes aligned and it is in target
     byte order */
    virtual iss::status read_mem(uint64_t addr, std::vector<uint8_t> &buf) = 0;

    /* Write memory, buf is 4-bytes aligned and it is in target
     byte order */
    virtual iss::status write_mem(uint64_t addr, const std::vector<uint8_t> &buf) = 0;

    /*================ Resume/Wait  ============================*/

    /* Resume from current address, if not supported it has to be figured out by
     * wait */
    inline iss::status resume_from_current(bool step, int sig, rp_thread_ref thread) {
        return resume_from_current(step, sig, thread, std::function<void(unsigned)>());
    }

    virtual iss::status resume_from_current(bool step, int sig, rp_thread_ref thread,
                                            std::function<void(unsigned)> stop_callback) = 0;

    /* Resume from specified address, if not supported it
     has to be figured out by wait */
    inline iss::status resume_from_addr(bool step, int sig, uint64_t addr, rp_thread_ref thread) {
        return resume_from_addr(step, sig, addr, thread, std::function<void(unsigned)>());
    }

    virtual iss::status resume_from_addr(bool step, int sig, uint64_t addr, rp_thread_ref thread,
                                         std::function<void(unsigned)> stop_callback) = 0;

    /* Wait function, wait_partial is called by the proxy with one
     tick intervals, so it allows to break into running
     target */

    /* Check for event and return. It allows proxy server to
     check messages from gdb allowing gdb to stop/kill target.
     Break and kill commands are generated by a human being so,
     the process can wait inside wait_partial with some substantial
     timeouts. It seems like 1s time will be highest acceptable value.

     In this case return value RP_TARGETRET_NOSUPP means, that
     response to previous resume was - 'not supported'. If this operation
     is not implemented by target, then it will return OK and
     implemeted will be 0.

     status_string is unchanged unless return value is OK and
     implemented is non 0 */
    virtual iss::status wait_non_blocking(bool &running) { return iss::NotSupported; }

    /* Wait for event, fill (null-terminated) status_string upon successful
     return, if there is not enough space for 'TAA... string' use
     'SAA' instead, status_sting_len is always > 3

     In this case return value RP_TARGETRET_NOSUPP means, that
     response to previous resume was - 'not supported'. If this operation
     is not implemented by target, then it will return OK and
     implemeted will be 0

     status_string is unchanged unless return value is OK and
     implemented is non 0 */
    virtual iss::status wait_blocking() = 0;

    /*============= Queries ===============================*/

    /* Bits of mask determine set of information about thread
     to be retrieved, results are put into info.  */
    virtual iss::status process_query(unsigned int &mask, const rp_thread_ref &arg, rp_thread_info &info) = 0;

    /* List threads. If first is non-zero then start from the first thread,
     otherwise start from arg, result points to array of threads to be
     filled out, result size is number of elements in the result,
     num points to the actual number of threads found, done is
     set if all threads are processed.  */
    virtual iss::status thread_list_query(int first, const rp_thread_ref &arg, std::vector<rp_thread_ref> &result,
                                          size_t max_num, size_t &num, bool &done) = 0;

    /* Query current thread id */
    virtual iss::status current_thread_query(rp_thread_ref &thread) = 0;

    /* Query offset of major sections in memory */
    virtual iss::status offsets_query(uint64_t &text, uint64_t &data, uint64_t &bss) = 0;

    /* Query crc32 of memory area */
    virtual iss::status crc_query(uint64_t addr, size_t len, uint32_t &val) = 0;

    /* Raw query, see gdb-XXXX/gdb/remote.c. we got buffer
     call this function. It is a responsibility of the target
     to fill out out_buf correctly in case of success.

     It is planned to have more more specific queries in
     the nearest future.  */
    virtual iss::status raw_query(std::string in_buf, std::string &out_buf) = 0;

    /*============ Breakpoints ===========================*/
    /**
     * add a breakpoint
     *
     * @param type      the type of the breakpoint: 0 -  sw exec, 1 - hw exec, 2 -
     * write watchpoint, 3 - access watchpoint
     * @param addr      address of the breakpoint
     * @param length    length of the range to check
     * @return iss:Ok if successful, iss::Err otherwise
     */
    virtual iss::status add_break(int type, uint64_t addr, unsigned int length) = 0;
    /**
     * remove a breakpoint
     *
     * @param type      the type of the breakpoint: 0 -  sw exec, 1 - hw exec, 2 -
     * write watchpoint, 3 - access watchpoint
     * @param addr      address of the breakpoint
     * @param length    length of the range to check
     * @return iss:Ok if successful, iss::Err otherwise
     */
    virtual iss::status remove_break(int type, uint64_t addr, unsigned int length) = 0;

    virtual iss::status add_break_condition(std::function<unsigned()> break_cond) = 0;

    /* Query thread info */
    virtual iss::status threadinfo_query(int first, std::string &out_buf) = 0;

    /* Query thread extra info */
    virtual iss::status threadextrainfo_query(const rp_thread_ref &thread, std::string &out_buf) = 0;

    /* Query packet size */
    virtual iss::status packetsize_query(std::string &out_buf) = 0;

    virtual iss::status target_xml_query(std::string &out_buf) { return iss::NotSupported; }
};

} // namespace debugger
} // namspace iss

#endif /* _TARGET_ADAPTER_H_ */
