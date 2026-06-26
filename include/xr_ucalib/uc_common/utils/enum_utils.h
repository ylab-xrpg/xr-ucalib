// Copyright 2026 Yongjiang Laboratory
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#pragma once

#include <boost/preprocessor.hpp>
#include <memory>
#include <stdexcept>

namespace xr_ucalib {

/**
 * @brief Custom macro for enum to/from string support. Only enum
 * structs/classes with consecutive indexes are supported.
 *
 * Example:
 * [Reference]: enum class MyEnum {C1, C2, C3};
 * [New code]: MAKE_ENUM_CLASS(MyEnum, 0, C1, C2, C3);
 *             MyEnumToString(MyEnum::C1);  -> "C1"
 *             MyEnumFromString("C2");      -> MyEnum::C2
 */

#define ENUM_TO_STRING_PROCESS_ELEMENT(r, start_idx, idx, elem) \
  case ((idx) + (start_idx)):                                   \
    return BOOST_PP_STRINGIZE(elem);
#define ENUM_FROM_STRING_PROCESS_ELEMENT(r, name, idx, elem) \
  if (str == BOOST_PP_STRINGIZE(elem)) {                     \
    return name::elem;                                       \
  }

#define DEFINE_ENUM_TO_FROM_STRING(name, start_idx, ...)                     \
  [[maybe_unused]] static std::string_view name##ToString(int value) {       \
    switch (value) {                                                         \
      BOOST_PP_SEQ_FOR_EACH_I(ENUM_TO_STRING_PROCESS_ELEMENT, start_idx,     \
                              BOOST_PP_VARIADIC_TO_SEQ(__VA_ARGS__));        \
      default:                                                               \
        throw std::runtime_error("Unknown value: " + std::to_string(value) + \
                                 " for enum: " + BOOST_PP_STRINGIZE(name));  \
    }                                                                        \
  }                                                                          \
  [[maybe_unused]] static std::string_view name##ToString(name value) {      \
    return name##ToString(static_cast<int>(value));                          \
  }                                                                          \
  [[maybe_unused]] static name name##FromString(std::string_view str) {      \
    BOOST_PP_SEQ_FOR_EACH_I(ENUM_FROM_STRING_PROCESS_ELEMENT, name,          \
                            BOOST_PP_VARIADIC_TO_SEQ(__VA_ARGS__));          \
    throw std::runtime_error("Unknown string value: " + std::string(str) +   \
                             " for enum: " + BOOST_PP_STRINGIZE(name));      \
  }

#define ENUM_PROCESS_ELEMENT(r, start_idx, idx, elem) \
  elem = (idx) + (start_idx),

#define ENUM_VALUES(start_idx, ...)                        \
  BOOST_PP_SEQ_FOR_EACH_I(ENUM_PROCESS_ELEMENT, start_idx, \
                          BOOST_PP_VARIADIC_TO_SEQ(__VA_ARGS__))

#define MAKE_ENUM(name, start_idx, ...)              \
  enum name { ENUM_VALUES(start_idx, __VA_ARGS__) }; \
  DEFINE_ENUM_TO_FROM_STRING(name, start_idx, __VA_ARGS__)

#define MAKE_ENUM_CLASS(name, start_idx, ...)              \
  enum class name { ENUM_VALUES(start_idx, __VA_ARGS__) }; \
  DEFINE_ENUM_TO_FROM_STRING(name, start_idx, __VA_ARGS__)

} // namespace xr_ucalib