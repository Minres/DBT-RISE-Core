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

#ifndef _ISS_PLUGIN_LOADER_H_
#define _ISS_PLUGIN_LOADER_H_

// standard library
#include <string>
#include <unordered_map>
#include <vector>
#include <stdexcept>
#include <memory>

#if defined(_WIN32) || defined(_WIN64)
#define OS_IS_WINDOWS true
#include <libloaderapi.h>
#else
#define OS_IS_WINDOWS false
#include <dlfcn.h>
#endif

namespace iss {
namespace plugin {
class loader {
public: // sub-definitions
    using function = void (*)();
    using variable = void *;

private: // sub-definitions

    struct symbol {
        bool is_func = false;
        union {
            variable var = nullptr;
            function func;
        };

        symbol() = default;

        symbol(function function)
        : is_func(true), func(function) { }

        symbol(variable var)
        : is_func(false), var(var) { }
    };

    struct plugin_data {
        void *handle = nullptr;
        std::string filepath;
        std::unordered_map<std::string, symbol> symbols;

        plugin_data(void *handle, std::string filepath)
        : handle(handle), filepath(filepath) { }

        ~plugin_data() {
            if (handle) {
#if OS_IS_WINDOWS
                FreeLibrary((HINSTANCE)_handle);
#else
                dlclose(handle);
#endif
            }
        }
    };

private: // members
    static std::unordered_map<std::string, std::shared_ptr<plugin_data>> _cache;
    std::shared_ptr<plugin_data> _data;
    std::shared_ptr<plugin_data> get_data(const std::string &filepath);

public: // methods

    loader(const std::string &filepath, const std::vector<std::string> &functions = {}, const std::vector<std::string> &variables = {})
    : _data(get_data(filepath)) {
        for (const auto &label : functions)
            bind_function(label);
        for (const auto &label : variables)
            bind_variable(label);
    }

    loader(loader &&other)
    : _data(std::move(other._data)) {
    }

    loader(const loader &other)
    : _data(_cache[other.filepath()]) {
    }

    void bind_function(const std::string &label);
    void bind_variable(const std::string &label);

    inline bool contains(const std::string &label) const {
        return _data->symbols.find(label) != _data->symbols.end();
    }

    inline bool contains_function(const std::string &label) const {
        auto iter = _data->symbols.find(label);
        return iter != _data->symbols.end() && iter->second.is_func;
    }

    inline bool contains_variable(const std::string &label) const {
        auto iter = _data->symbols.find(label);
        return iter != _data->symbols.end() && !iter->second.is_func;
    }

    function get_function_ptr(const std::string &label) const;
    variable get_variable_ptr(const std::string &label) const;

    template<typename T>
    inline T get_variable(const std::string &label) const {
        return *(T*) get_variable_ptr(label);
    }

    template<typename T, typename ... Args>
    inline T call_function(const std::string &label, Args ... args) const {
        T (*func)(Args...) = (decltype(func))get_function_ptr(label);
        return func(args...);
    }

    inline loader& operator=(const loader &other) {
        _data = _cache[other.filepath()];
        return *this;
    }

    inline loader& operator=(loader &&other) {
        _data = std::move(other._data);
        return *this;
    }

    inline std::shared_ptr<loader::plugin_data> data() const {
        return _data;
    }

    inline size_t count() const {
        return _data->symbols.size();
    }

    inline const std::string& filepath() const {
        return _data->filepath;
    }
};
}
}
#endif /* _ISS_PLUGIN_LOADER_H_ */
