/*******************************************************************************
 * Copyright (C) 2021, MINRES Technologies GmbH
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
#ifndef _ISS_PLUGIN_CALCULATOR_H_
#define _ISS_PLUGIN_CALCULATOR_H_

#include <functional>
#include <memory>
#include <string>
#include <unordered_map>
#include <vector>

namespace iss {
namespace plugin {

class calculator {
public:
    calculator(uint32_t* reg_base_ptr, std::string const& formula);
    ~calculator();

    calculator(const calculator&);

    unsigned operator()(uint64_t instr);

private:
    std::string error;
    std::vector<int> byte_code;
    std::vector<std::function<unsigned(uint64_t)>> var_accessors;
    std::vector<int> stack{1024};
    std::vector<int>::iterator stack_ptr{stack.begin()};
    uint32_t* reg_base_ptr{nullptr};
};
} // namespace plugin
} // namespace iss
#endif /* _ISS_PLUGIN_CALCULATOR_H_ */
