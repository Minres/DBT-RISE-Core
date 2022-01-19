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

#include "iss/debugger/encoderdecoder.h"

#include <iomanip>
#include <sstream>
#include <string>
#include <util/logging.h>

using namespace iss::debugger;

enum {
    // breakpoint types
    BPTYPE_MIN = 0,
    BPTYPE_MAX = 4,
    //_query mask bits
    MASKBIT_THREADID = 1,
    MASKBIT_EXISTS = 2,
    MASKBIT_DISPLAY = 4,
    MASKBIT_THREADNAME = 8,
    MASKBIT_MOREDISPLAY = 16
};

/* Convert stream of chars into data */
std::vector<uint8_t> encoder_decoder::dec_data(const char *in) {
    size_t count;
    unsigned int bytex;
    std::vector<uint8_t> out;
    assert(in != nullptr);

    for (count = 0; *in; count++, in += 2) {
        if (*(in + 1) == '\0') {
            /* Odd number of nibbles. Discard the last one */
            LOG(WARN) << __FUNCTION__ << ": odd number of nibbles";
            if (count == 0) out.clear();
            return out;
        }

        if (!dec_byte(in, &bytex)) { // parse error
            out.clear();
            return out;
        }
        out.push_back(bytex & 0xff);
    }

    if (*in) // Input too long
        out.clear();
    return out;
}

int encoder_decoder::dec_reg(const char *in, unsigned int *reg_no) {
    if (!dec_uint32(&in, reg_no, '\0')) return false;

    return true;
}

/* Decode reg_no=XXXXXX */
std::vector<uint8_t> encoder_decoder::dec_reg_assignment(const char *in, unsigned int *reg_no) {
    assert(in != nullptr);
    assert(reg_no != nullptr);

    if (!dec_uint32(&in, reg_no, '=')) return std::vector<uint8_t>();

    return dec_data(in);
}

/* Decode memory transfer parameter in the form of AA..A,LL..L */
int encoder_decoder::dec_mem(const char *in, uint64_t *addr, size_t *len) {
    assert(in != nullptr);
    assert(addr != nullptr);
    assert(len != nullptr);
    if (!dec_uint64(&in, addr, ',')) return false;

    *len = 0;
    return dec_uint32(&in, (unsigned *)len, '\0');
}

/* Decode process query. Format: 'MMMMMMMMRRRRRRRRRRRRRRRR'
 where:
 M represents mask
 R represents thread reference */
int encoder_decoder::dec_process_query(const char *in, unsigned int *mask, rp_thread_ref *ref) {
    unsigned int tmp_mask;
    uint64_t tmp_val;

    assert(in != nullptr);
    assert(mask != nullptr);
    assert(ref != nullptr);

    if (!dec_4bytes(in, &tmp_mask)) return false;
    in += 8;

    if (!dec_8bytes(in, &tmp_val)) return false;

    *mask = tmp_mask;
    ref->val = tmp_val;

    return true;
}

/* Decode thread list list query. Format 'FMMAAAAAAAAAAAAAAAA'
 where:
 F represents first flag
 M represents max count
 A represents argument thread reference */
int encoder_decoder::dec_list_query(const char *in, int *first, size_t *max, rp_thread_ref *arg) {
    unsigned int first_flag;
    size_t tmp_max;
    uint64_t tmp_val;

    assert(in != nullptr);
    assert(first != nullptr);
    assert(max != nullptr);
    assert(arg != nullptr);

    if (!dec_nibble(in, &first_flag)) return false;
    in++;

    if (!dec_byte(in, (unsigned *)&tmp_max)) return false;
    in += 2;

    if (!dec_8bytes(in, &tmp_val)) return false;

    *first = (first_flag) ? true : false;
    *max = tmp_max;
    arg->val = tmp_val;

    return true;
}

/* Decode a breakpoint (z or Z) packet */
int encoder_decoder::dec_break(const char *in, int *type, uint64_t *addr, unsigned int *len) {
    unsigned int val;

    assert(in != nullptr);
    assert(*in != '\0');
    assert(type != nullptr);
    assert(addr != nullptr);
    assert(len != nullptr);

    in++;
    if (!dec_nibble(in, &val)) return false;
    in++;

    if (val < BPTYPE_MIN || val > BPTYPE_MAX) return false;

    if (*in++ != ',') return false;

    *type = val;

    if (!dec_uint64(&in, addr, ',')) return false;

    if (!dec_uint32(&in, len, '\0')) return false;

    return true;
}

/* If a byte of avail is 0 then the corresponding data byte is
 encoded as 'xx', otherwise it is encoded in normal way */
std::string encoder_decoder::enc_regs(const std::vector<uint8_t> &data, const std::vector<uint8_t> &avail) {
    assert(data.size() > 0);

    std::stringstream ss;
    for (size_t i = 0; i < data.size(); i++) {
        if (avail[i]) {
            ss << enc_byte(data[i], true) << enc_byte(data[i], false);
        } else {
            ss << "xx";
        }
    }
    return ss.str();
}

/* Convert an array of bytes into an array of characters */
std::string encoder_decoder::enc_data(const unsigned char *data, size_t data_len) {
    assert(data != nullptr);
    assert(data_len > 0);

    std::stringstream ss;
    for (size_t i = 0; i < data_len; i++, data++) ss << enc_byte(*data, true) << enc_byte(*data, false);
    return ss.str();
}

std::string encoder_decoder::enc_data(const std::vector<uint8_t> &data) {
    assert(data.size() > 0);

    std::stringstream ss;
    for (unsigned char i : data) ss << enc_byte(i, true) << enc_byte(i, false);
    return ss.str();
}
/* Encode string into an array of characters */
int encoder_decoder::enc_string(const char *s, char *out, size_t out_size) {
    int i;
    const char hex[] = "0123456789abcdef";

    assert(s != nullptr);
    assert(out != nullptr);
    assert(out_size > 0);

    if (strlen(s) * 2 >= out_size) {
        /* We do not have enough space to encode the data */
        return false;
    }

    i = 0;
    while (*s) {
        *out++ = hex[(*s >> 4) & 0x0f];
        *out++ = hex[*s & 0x0f];
        s++;
        i++;
    }
    *out = '\0';
    return i;
}

void encoder_decoder::encode_str(std::stringstream &ss, const std::string &str) {
    /* and Encode value */
    std::ios init(nullptr);
    init.copyfmt(ss);
    ss << std::setw(2) << std::setfill('0') << std::hex << str.length();
    ss.copyfmt(init);
    ss << str;
}

/* Encode result of process query:
 qQMMMMMMMMRRRRRRRRRRRRRRRR(TTTTTTTTLLVV..V)*,
 where
 M   represents mask
 R   represents ref
 T   represents tag
 L   represents length
 V   represents value */
std::string encoder_decoder::enc_process_query_response(unsigned int mask, const rp_thread_ref *ref,
                                                        const rp_thread_info *info) {
    size_t len;
    unsigned int tag;
    int i;
    assert(ref != nullptr);
    assert(info != nullptr);

    std::stringstream ss;

    /* Encode header */
    ss << "qQ";
    /* Encode mask */
    ss << std::setw(8) << std::setfill('0') << std::hex << mask;

    /* Encode reference thread */
    ss << std::setw(16) << std::setfill('0') << std::hex << ref->val;

    for (i = 0, tag = 0; i < 32; i++, tag <<= 1) {
        if ((mask & tag) == 0) continue;
        /* Encode tag */
        ss << std::setw(8) << std::setfill('0') << std::hex << tag;
        switch (tag) {
        case MASKBIT_THREADID:
            /* Encode length - it is 16 */
            /* and Encode value */
            ss << "10" << std::setw(16) << std::setfill('0') << std::hex << info->thread_id.val;
            break;
        case MASKBIT_EXISTS:
            /* Encode Length */
            /* and Encode value */
            ss << "01" << ((info->exists) ? '1' : '0');
            break;
        case MASKBIT_DISPLAY:
            /* Encode length */
            /* and Encode value */
            encode_str(ss, info->display);
            break;
        case MASKBIT_THREADNAME:
            /* Encode length */
            /* Encode value */
            encode_str(ss, info->thread_name);
            break;
        case MASKBIT_MOREDISPLAY:
            /* Encode length */
            /* Encode value */
            encode_str(ss, info->more_display);
            break;
        default:
            /* Unexpected tag value */
            assert(false);
            return nullptr;
        }
    }

    return ss.str();
}

/* Encode result of list query:
 qMCCDAAAAAAAAAAAAAAAA(FFFFFFFFFFFFFFFF)*,
 where
 C   reprsents  count
 D   represents done
 A   represents arg thread reference
 F   represents found thread reference(s) */
std::string encoder_decoder::enc_list_query_response(size_t count, int done, const rp_thread_ref &arg,
                                                     const std::vector<rp_thread_ref> found) {
    assert(count <= 255);

    std::string enc_buf;
    std::stringstream ss;

    /* Encode header, count, done and arg */
    ss << "qM";
    enc_byte(count, enc_buf);
    ss << enc_buf[0] << enc_buf[1];
    ss << ((done) ? '1' : '0');
    ss << std::setw(16) << std::setfill('0') << std::hex << arg.val;

    /* Encode found */
    for (int i = 0; i < count; i++) {
        ss << std::setw(16) << std::setfill('0') << std::hex << found[i].val;
    }
    return ss.str();
}

/* Decode a single nibble */
int encoder_decoder::dec_nibble(const char *in, unsigned int *nibble) {
    int nib = -1;

    if (*in >= '0' && *in <= '9') nib = *in - '0';
    if (*in >= 'A' && *in <= 'F') nib = *in - 'A' + 10;
    if (*in >= 'a' && *in <= 'f') nib = *in - 'a' + 10;

    if (nib >= 0) {
        *nibble = nib;
        return true;
    }

    return false;
}

/* Decode byte */
int encoder_decoder::dec_byte(const char *in, unsigned int *byte_ptr) {
    unsigned int ls_nibble;
    unsigned int ms_nibble;

    if (!dec_nibble(in, &ms_nibble)) return false;

    if (!dec_nibble(in + 1, &ls_nibble)) return false;

    *byte_ptr = (ms_nibble << 4) + ls_nibble;
    return true;
}

/* Decode exactly 4 bytes of hex from a longer string, and return the result
 as an unsigned 32-bit value */
int encoder_decoder::dec_4bytes(const char *in, uint32_t *val) {
    unsigned int nibble;
    uint32_t tmp;
    int count;

    for (tmp = 0, count = 0; count < 8; count++, in++) {
        if (!dec_nibble(in, &nibble)) break;
        tmp = (tmp << 4) + nibble;
    }
    *val = tmp;
    return true;
}

/* Decode exactly 8 bytes of hex from a longer string, and return the result
 as an unsigned 64-bit value */
int encoder_decoder::dec_8bytes(const char *in, uint64_t *val) {
    unsigned int nibble;
    uint64_t tmp;
    int count;

    for (tmp = 0, count = 0; count < 16; count++, in++) {
        if (!dec_nibble(in, &nibble)) break;
        tmp = (tmp << 4) + nibble;
    }
    *val = tmp;
    return true;
}

/* Decode a hex string to an unsigned 32-bit value */
int encoder_decoder::dec_uint32(const char **in, uint32_t *val, char break_char) {
    unsigned int nibble;
    uint32_t tmp;
    int count;

    assert(in != nullptr);
    assert(val != nullptr);

    if (**in == '\0') { // We are expecting at least one character
        return false;
    }

    for (tmp = 0, count = 0; **in && count < 8; count++, (*in)++) {
        if (!dec_nibble(*in, &nibble)) break;
        tmp = (tmp << 4) + nibble;
    }

    if (**in != break_char) { // Wrong terminating character
        return false;
    }
    if (**in) (*in)++;
    *val = tmp;
    return true;
}

/* Decode a hex string to an unsigned 64-bit value */
int encoder_decoder::dec_uint64(const char **in, uint64_t *val, char break_char) {
    unsigned int nibble;
    uint64_t tmp;
    int count;

    assert(in != nullptr);
    assert(val != nullptr);

    if (**in == '\0') {
        /* We are expecting at least one character */
        return false;
    }

    for (tmp = 0, count = 0; **in && count < 16; count++, (*in)++) {
        if (!dec_nibble(*in, &nibble)) break;
        tmp = (tmp << 4) + nibble;
    }

    if (**in != break_char) {
        /* Wrong terminating character */
        return false;
    }
    if (**in) (*in)++;
    *val = tmp;
    return true;
}

/* Encode byte */
void encoder_decoder::enc_byte(unsigned char val, std::string &out, size_t offset) {
    const char hex[] = "0123456789abcdef";
    out.reserve(offset + 2);
    out[offset] = hex[(val >> 4) & 0xf];
    out[offset + 1] = hex[val & 0xf];
}
