/*******************************************************************************
 * Copyright (C) 2017, MINRES Technologies GmbH
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

#ifndef _RANGE_LUT_H_
#define _RANGE_LUT_H_

#include <map>
#include <stdexcept>
#include <string>
#include <iostream>
#include <iomanip>
#include <exception>
#include <loki/AssocVector.h>

namespace util {

template<typename T>
class range_lut{
    enum entry_type {BEGIN_RANGE=1, END_RANGE=2, SINGLE_BYTE_RANGE=3};

    struct lut_entry {
        T index;
        entry_type type;
    };

public:
    /**
     * constructor or the lookup table
     * @param null_entry the entry to be used for empty slots
     */
    range_lut(T null_entry):null_entry(null_entry){ }
    /**
     * add an T to the lut covering the range starting at base_addr until base_addr+size-1
     * @param i the entry
     * @param base_addr the base address
     * @param size the size of the occupied range
     */
    void addEntry(T i, uint64_t base_addr, uint64_t size);
    /**
     * remove an entry with value i of type T
     * @param i the entry to be found
     * @return true if the entry is found and removed, false otherwise
     */
    bool removeEntry(T i);
    /**
     * get number of entries in the lookup table
     * @return the size of the underlying container
     */
    size_t size(){
        return lut.size();
    }
    /**
     * remove all entries from the lut
     */
    void clear(){
        lut.clear();
    }
    /**
     * get the entry T associated with a given address
     * @param addr the address
     * @return the entry belonging to the address
     */
    inline
    T getEntry(uint64_t addr){
        auto iter = lut.lower_bound(addr);
        return (iter!=lut.end() && (iter->second.type==END_RANGE ||iter->first==addr))? iter->second.index:null_entry;
    }
    /**
     * validate the lookup table wrt. overlaps
     */
    void validate() ;
    /**
     * create a textual representation of the address map (address range->entry association)
     * @return
     */
    std::string toString();
    /**
     * the null entry
     */
    const  T null_entry;
protected:
    Loki::AssocVector<uint64_t, lut_entry> lut;
};

/**
 * overloaded stream operator
 * @param os the output stream
 * @return the stream
 */
template<typename T>
std::ostream& operator <<(std::ostream& os, range_lut<T>& lut) {
    os << lut.toString();
    return os;
}

template<typename T>
inline void range_lut<T>::addEntry(T i, uint64_t base_addr, uint64_t size) {
    auto iter = lut.find(base_addr);
    if (iter != lut.end() && iter->second.index != null_entry) throw std::runtime_error("range already mapped");

    uint64_t eaddr = base_addr + size - 1;
    if (eaddr < base_addr) throw std::runtime_error("address wrap-around occurred");

    lut[base_addr] = lut_entry { i, size > 1 ? BEGIN_RANGE : SINGLE_BYTE_RANGE };
    if (size > 1) lut[eaddr] = lut_entry { i, END_RANGE };
}

template<typename T>
inline bool range_lut<T>::removeEntry(T i) {
    throw std::exception();
//    auto iter = rlut.find(i);
//    if (iter != rlut.end()) {
//        uint64_t baddr = iter->second;
//        auto beg_iter = lut.find(baddr);
//        if (beg_iter->second.type == SINGLE_BYTE_RANGE) {
//            lut.erase(beg_iter);
//        } else {
//            auto end_iter = beg_iter;
//            ++end_iter;
//            lut.erase(beg_iter, ++end_iter);
//        }
//        rlut.erase(iter);
//        return true;
//    }
    return false;
}

template<typename T>
inline void range_lut<T>::validate() {
    bool mapped = false;
    for (auto iter = lut.begin(); iter != lut.end(); iter++) {
        switch (iter->second.type) {
        case SINGLE_BYTE_RANGE:
            if (iter->second.index != null_entry && mapped) throw std::runtime_error("range overlap: begin range while in mapped range");

            break;
        case BEGIN_RANGE:
            if (iter->second.index != null_entry) {
                if (mapped) {
                    throw std::runtime_error("range overlap: begin range while in mapped range");
                }
                mapped = true;
            }
            break;
        case END_RANGE:
            if (!mapped) {
                throw std::runtime_error("range overlap: end range while in unmapped region");
            }
            mapped = false;
            break;
        }
    }
}

template<typename T>
inline std::__cxx11::string range_lut<T>::toString() {
    std::ostringstream buf;
    for (auto iter = lut.begin(); iter != lut.end(); ++iter) {
        switch (iter->second.type) {
        case BEGIN_RANGE:
            if (iter->second.index != null_entry) {
                buf << "  from 0x" << std::setw(sizeof(uint64_t) * 2) << std::setfill('0') << std::uppercase << std::hex << iter->first
                        << std::dec;
            }
            break;
        case END_RANGE:
            buf << " to 0x" << std::setw(sizeof(uint64_t) * 2) << std::setfill('0') << std::uppercase << std::hex << iter->first << std::dec
                    << " as " << iter->second->index << std::endl;
        }
    }
    return buf.str();
}


}  // namespace util


#endif /* _RANGE_LUT_H_ */
