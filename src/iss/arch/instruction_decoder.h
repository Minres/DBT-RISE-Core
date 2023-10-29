/*
 * instruction_decoder.h
 *
 *  Created on: Jun 19, 2022
 *      Author: eyck
 */

#ifndef _ISS_ARCH_INSTRUCTION_DECODER_H_
#define _ISS_ARCH_INSTRUCTION_DECODER_H_

#include <memory>

namespace iss {
namespace arch {

template <typename ARCH> struct instruction_decoder {

    static std::unique_ptr<instruction_decoder<ARCH>> create();

    template <typename T> unsigned decode_instruction(T);
};
} // namespace arch
} // namespace iss

#endif /* _ISS_ARCH_INSTRUCTION_DECODER_H_ */
