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

#include <algorithm>
#include <cassert>
#include <cstddef>
#include <cstdint>
#include <iss/instruction_decoder.h>
#include <limits>
#include <numeric>
#include <vector>

using namespace iss;

decoder::decoder(std::vector<generic_instruction_descriptor> instr_list) {
    for(auto instr : instr_list) {
        root.instrs.push_back(instr);
    }
    populate_decoding_tree(root);
}

void decoder::populate_decoding_tree(decoding_tree_node& parent) {
    // create submask
    parent.submask =
        std::accumulate(parent.instrs.begin(), parent.instrs.end(), std::numeric_limits<uint32_t>::max(),
                        [](int current_submask, const generic_instruction_descriptor& instr) { return current_submask & instr.mask; });
    //  put each instr according to submask&encoding into children
    for(auto instr : parent.instrs) {
        bool foundMatch = false;
        for(auto& child : parent.children) {
            // use value as identifying trait
            if(child.value == (instr.value & parent.submask)) {
                child.instrs.push_back(instr);
                foundMatch = true;
            }
        }
        if(!foundMatch) {
            decoding_tree_node child = decoding_tree_node(instr.value & parent.submask);
            child.instrs.push_back(instr);
            parent.children.push_back(child);
        }
    }
    parent.instrs.clear();
    // call populate_decoding_tree for all children
    if(parent.children.size() > 1)
        for(auto& child : parent.children) {
            populate_decoding_tree(child);
        }
    else {
        // sort instrs by value of the mask, so we have the least restrictive mask last
        std::sort(parent.children[0].instrs.begin(), parent.children[0].instrs.end(),
                  [](const generic_instruction_descriptor& instr1, const generic_instruction_descriptor& instr2) {
                      return instr1.mask > instr2.mask;
                  });
    }
}
uint32_t decoder::decode_instr(uint32_t word) { return _decode_instr(this->root, word); }
uint32_t decoder::_decode_instr(decoding_tree_node const& node, uint32_t word) {
    if(!node.instrs.empty()) {
        for(auto& instr : node.instrs) {
            if((instr.mask & word) == instr.value)
                return instr.index;
        }
    } else if(!node.children.empty()) {
        for(auto& child : node.children) {
            if(child.value == (node.submask & word)) {
                return _decode_instr(child, word);
            }
        }
    }
    return std::numeric_limits<uint32_t>::max();
}
