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
// parts of the code are https://github.com/ikehirzel/cxx-plugin
/**
 * Copyright 2020 Ike Hirzel
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in the
 * Software without restriction, including without limitation the rights to use, copy,
 * modify, merge, publish, distribute, sublicense, and/or sell copies of the Software,
 * and to permit persons to whom the Software is furnished to do so, subject to the
 * following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED,
 * INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A
 * PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT
 * HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION
 * OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE
 * SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 */

#include <iss/plugin/loader.h>

using namespace iss::plugin;
std::unordered_map<std::string, std::shared_ptr<loader::plugin_data>> loader::_cache;

std::shared_ptr<loader::plugin_data> loader::get_data(const std::string& filepath) {
    if (filepath.empty())
        throw std::invalid_argument("failed to bind to loader: filepath was empty");
    auto iter = _cache.find(filepath);
    if (iter != _cache.end())
        return iter->second;
#if OS_IS_WINDOWS
    auto handle = (void*)LoadLibrary(filepath.c_str());
    if (!handle)
        throw std::invalid_argument("failed to bind to loader '"
                + filepath
                + "': ERROR "
                + std::to_string(GetLastError()));
#else
    auto handle = dlopen(filepath.c_str(), RTLD_NOW);
    if(!handle)
        throw std::invalid_argument("failed to bind to loader '"
                + filepath
                + "': "
                + std::string(dlerror()));
#endif
    auto data = std::make_shared<plugin_data>(handle, filepath);
    _cache[filepath] = data;
    return data;
}

void loader::bind_function(const std::string& label) {
    if (_data->symbols.find(label) != _data->symbols.end())
        return;
#if OS_IS_WINDOWS
    Function ptr = (Function)GetProcAddress((HMODULE)_handle, label.c_str());
    if (!ptr)
        throw std::invalid_argument("failed to bind variable '"
                + label
                + "': ERROR "
                + std::to_string(GetLastError()));
#else
    function ptr = (function)dlsym(_data->handle, label.c_str());
    if (!ptr)
        throw std::invalid_argument("failed to bind function '"
                + label
                + "': "
                + std::string(dlerror()));
#endif
    if (!ptr)
        throw std::invalid_argument("failed to bind function '"
                + label
                + "': symbol could not be found");
    _data->symbols[label] = symbol(ptr);
}

void loader::bind_variable(const std::string& label) {
    if (_data->symbols.find(label) != _data->symbols.end())
        return;
#if OS_IS_WINDOWS
    Variable ptr = (Variable)GetProcAddress((HMODULE)_handle, label.c_str());
    if (!ptr)
        throw std::invalid_argument("failed to bind variable '"
                + label
                + "': ERROR "
                + std::to_string(GetLastError()));
#else
    variable ptr = (variable)dlsym(_data->handle, label.c_str());
    if (!ptr)
        throw std::invalid_argument("failed to bind variable '"
                + label
                + "': "
                + std::string(dlerror()));
#endif
    _data->symbols[label] = symbol(ptr);
}

loader::variable loader::get_variable_ptr(const std::string& label) const {
    auto iter = _data->symbols.find(label);
    if (iter == _data->symbols.end())
        throw std::invalid_argument("failed to get variable pointer '"
                + label
                + "': symbol is not bound");
    if (iter->second.is_func)
        throw std::invalid_argument("failed to get variable pointer '"
                + label
                + "': symbol does not reference a variable");
    return iter->second.var;
}

loader::function loader::get_function_ptr(const std::string& label) const {
    auto iter = _data->symbols.find(label);
    if (iter == _data->symbols.end())
        throw std::invalid_argument("failed to get function '"
                + label
                + "': symbol is not bound");
    if (!iter->second.is_func)
        throw std::invalid_argument("symbol '"
                + label
                + "' does not reference a function");
    return iter->second.func;
}
