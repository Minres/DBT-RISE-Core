/*******************************************************************************
 * Copyright (C) 2024 MINRES Technologies GmbH
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
 *       alex@minres.com - initial implementation
 ******************************************************************************/

#ifndef _INSTR_DECODER_H
#define _INSTR_DECODER_H
#include <cstddef>
#include <cstdint>
#include <limits>
#include <vector>
namespace iss {

enum { DECODING_FAIL = std::numeric_limits<uint32_t>::max() };

struct generic_instruction_descriptor {
    uint32_t value;
    uint32_t mask;
    uint32_t index;
};

struct decoding_tree_node {
    std::vector<generic_instruction_descriptor> instrs;
    std::vector<decoding_tree_node> children;
    uint32_t submask = std::numeric_limits<uint32_t>::max();
    uint32_t value;
    decoding_tree_node(uint32_t value)
    : value(value) {}
};
class decoder {
public:
    decoder(std::vector<generic_instruction_descriptor> const& instr_list);
    uint32_t decode_instr(uint32_t word);

private:
    decoding_tree_node root{decoding_tree_node(std::numeric_limits<uint32_t>::max())};
    void populate_decoding_tree(decoding_tree_node& root);
    uint32_t _decode_instr(decoding_tree_node const& node, uint32_t word);
};
} // namespace iss
#endif