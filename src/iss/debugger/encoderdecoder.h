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
#ifndef _ENCODERDECODER_H_
#define _ENCODERDECODER_H_
#include <cstddef>
#include <cstdint>
#include <iostream>
#include <string>
#include <vector>

#include "target_adapter_if.h"

namespace iss {
namespace debugger {

class encoder_decoder {
public:
    /* Decode/encode functions */
    std::vector<uint8_t> dec_data(const char *in);

    int dec_reg(const char *in, unsigned int *reg_no);

    std::vector<uint8_t> dec_reg_assignment(const char *in, unsigned int *reg_no);
    int dec_mem(const char *in, uint64_t *addr, size_t *len);
    int dec_process_query(const char *in, unsigned int *mask, rp_thread_ref *ref);
    int dec_break(const char *in, int *type, uint64_t *addr, unsigned int *len);
    int dec_list_query(const char *in, int *first, size_t *max, rp_thread_ref *arg);
    std::string enc_regs(const std::vector<uint8_t> &data, const std::vector<uint8_t> &avail);
    std::string enc_data(const unsigned char *data, size_t data_len);
    std::string enc_data(const std::vector<uint8_t> &data);
    int enc_string(const char *s, char *out, size_t out_size);
    std::string enc_process_query_response(unsigned int mask, const rp_thread_ref *ref, const rp_thread_info *info);
    std::string enc_list_query_response(size_t count, int done, const rp_thread_ref &arg,
                                        const std::vector<rp_thread_ref> found);
    int dec_nibble(const char *in, unsigned int *nibble);
    int dec_byte(const char *in, unsigned int *byte_ptr);
    int dec_4bytes(const char *in, uint32_t *val);
    int dec_8bytes(const char *in, uint64_t *val);
    int dec_uint32(const char **in, uint32_t *val, char break_char = 0);
    int dec_uint32(const char *in, uint32_t *val, char break_char = 0) { return dec_uint32(&in, val, break_char); }
    int dec_uint64(const char **in, uint64_t *val, char break_char = 0);
    int dec_uint64(const char *in, uint64_t *val, char break_char = 0) { return dec_uint64(&in, val, break_char); }
    void enc_byte(unsigned char val, std::string &out, size_t offset = 0);
    char enc_byte(unsigned char val, bool highNibble) { return highNibble ? hex[(val >> 4) & 0xf] : hex[val & 0xf]; }

private:
    const char *hex = "0123456789abcdef";
    void encode_str(std::stringstream &ss, const std::string &str);
};

} // namespace debugger
} // namspace iss
#endif /* _ENCODERDECODER_H_ */
