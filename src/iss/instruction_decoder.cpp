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
#include <sstream>
#include <vector>
#include <util/ities.h>

using namespace iss;

namespace {
constexpr std::size_t json_indent_width = 2;

void append_indent(std::ostream& os, std::size_t depth) {
    for(std::size_t i = 0; i < depth * json_indent_width; ++i)
        os.put(' ');
}

void append_instruction_json(std::ostream& os, generic_instruction_descriptor const& instr) {
    os << std::hex << "{\"value\": 0x" << instr.value << ",\"mask\": 0x" << instr.mask << std::dec << ",\"index\":" << instr.index << "}";
}

void append_node_json(std::ostream& os, decoding_tree_node const& node) {
    os << "{\"value\":" << node.value << ",\"submask\":" << node.submask << ",\"instrs\":[";
    for(std::size_t i = 0; i < node.instrs.size(); ++i) {
        if(i != 0)
            os << ',';
        append_instruction_json(os, node.instrs[i]);
    }
    os << "],\"children\":[";
    for(std::size_t i = 0; i < node.children.size(); ++i) {
        if(i != 0)
            os << ',';
        append_node_json(os, node.children[i]);
    }
    os << "]}";
}

void append_instruction_pretty_json(std::ostream& os, generic_instruction_descriptor const& instr, std::size_t depth) {
    append_indent(os, depth);
    os << "{\n";
    append_indent(os, depth + 1);
    os << "\"value\": \"0x" << std::hex << instr.value << "\",\n";
    append_indent(os, depth + 1);
    os << "\"mask\": \"0x" << instr.mask << std::dec << "\",\n";
    append_indent(os, depth + 1);
    os << "\"index\": " << instr.index << '\n';
    append_indent(os, depth);
    os << '}';
}

void append_node_pretty_json(std::ostream& os, decoding_tree_node const& node, std::size_t depth) {
    append_indent(os, depth);
    os << "{\n";
    append_indent(os, depth + 1);
    os << "\"value\": \"0x" << std::hex << node.value << "\",\n";
    append_indent(os, depth + 1);
    os << "\"submask\": \"0x" << node.submask << std::dec << "\",\n";
    append_indent(os, depth + 1);
    os << "\"instrs\": [";
    if(!node.instrs.empty()) {
        os << '\n';
        for(std::size_t i = 0; i < node.instrs.size(); ++i) {
            if(i != 0)
                os << ",\n";
            append_instruction_pretty_json(os, node.instrs[i], depth + 2);
        }
        os << '\n';
        append_indent(os, depth + 1);
    }
    os << "],\n";
    append_indent(os, depth + 1);
    os << "\"children\": [";
    if(!node.children.empty()) {
        os << '\n';
        for(std::size_t i = 0; i < node.children.size(); ++i) {
            if(i != 0)
                os << ",\n";
            append_node_pretty_json(os, node.children[i], depth + 2);
        }
        os << '\n';
        append_indent(os, depth + 1);
    }
    os << "]\n";
    append_indent(os, depth);
    os << '}';
}

void populate_decoding_tree(decoding_tree_node& parent) {
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
                      return util::bit_count(instr1.mask) > util::bit_count(instr2.mask);
                  });
    }
}

uint32_t decode_instr_rec(decoding_tree_node const& node, uint32_t word) {
    if(!node.instrs.empty()) {
        for(auto& instr : node.instrs) {
            if((instr.mask & word) == instr.value)
                return instr.index;
        }
    } else if(!node.children.empty()) {
        for(auto& child : node.children) {
            if(child.value == (node.submask & word)) {
                return decode_instr_rec(child, word);
            }
        }
    }
    return std::numeric_limits<uint32_t>::max();
}

void consolidate_decoding_tree(decoding_tree_node& node) {
    if (!node.children.empty()) {
        auto can_consolidate = true;
        for (auto &child : node.children) {
            if (node.children.empty() &&(child.instrs.empty() || child.instrs.size() > 1))
                can_consolidate = false;
        }
        node.instrs.reserve(node.children.size());
        if (can_consolidate) {
            for (auto& child : node.children) {
                node.instrs.emplace_back(std::move(child.instrs.front()));
            }
            node.children.clear();
        } else {
            for (auto &child : node.children)
                consolidate_decoding_tree(child);
        }
    }
}

} // namespace

decoder::decoder(std::vector<generic_instruction_descriptor> const& instr_list) {
    for(auto instr : instr_list) {
        root.instrs.push_back(instr);
    }
    // sort instrs by number of 1s in the mask, so we have the least restrictive mask last
    // if masks are the same the number of 1s in the value is used
//    std::sort(root.instrs.begin(), root.instrs.end(),
//            [](const generic_instruction_descriptor &instr1, const generic_instruction_descriptor &instr2) {
//                auto bcm1 = util::bit_count(instr1.mask);
//                auto bcm2 = util::bit_count(instr2.mask);
//                return bcm1 != bcm2 ? bcm1 > bcm2 : util::bit_count(instr1.value) > util::bit_count(instr2.value);
//            });
    populate_decoding_tree(root);
    // now remove unneeded decoding_tree_node
//    consolidate_decoding_tree(root);
}

uint32_t decoder::decode_instr(uint32_t word) { return decode_instr_rec(this->root, word); }

std::string decoder::print_tree_as_json() const {
    std::ostringstream os;
    append_node_json(os, root);
    return os.str();
}

std::string decoder::print_tree_as_pretty_json() const {
    std::ostringstream os;
    append_node_pretty_json(os, root, 0);
    return os.str();
}
