// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2014 projectchrono.org
// All rights reserved.
//
// Use of this source code is governed by a BSD-style license that can be found
// in the LICENSE file at the top level of the distribution and at
// http://projectchrono.org/license-chrono.txt.
//
// =============================================================================
// Authors: Alessandro Tasora
// =============================================================================

#ifndef CH_UPDATE_FLAGS_H
#define CH_UPDATE_FLAGS_H

#include "chrono/core/ChApiCE.h"

namespace chrono {


/// Flags used when triggering updates in ChObject,  ChIntegrable, etc.
/// These flags can be combined with bitwise operators.
///     For example:
///       UpdateFlag flags = UpdateFlag::DYNAMICS | UpdateFlag::JACOBIANS;
/// 	  if (has_flag(flags, UpdateFlag::DYNAMICS)) { ... }
   
enum class UpdateFlag : uint8_t {
    NONE = 0u,
    DYNAMICS        = 1u << 0,
    JACOBIANS       = 1u << 1,
    VISUAL_ASSETS   = 1u << 2,
    UPDATE_ALL           = DYNAMICS | JACOBIANS | VISUAL_ASSETS,
    UPDATE_ALL_NO_VISUAL = DYNAMICS | JACOBIANS
};

constexpr UpdateFlag operator|(UpdateFlag lhs, UpdateFlag rhs) noexcept {
    using underlying = std::underlying_type_t<UpdateFlag>;
    return static_cast<UpdateFlag>(static_cast<underlying>(lhs) | static_cast<underlying>(rhs));
}
constexpr UpdateFlag operator&(UpdateFlag lhs, UpdateFlag rhs) noexcept {
    using underlying = std::underlying_type_t<UpdateFlag>;
    return static_cast<UpdateFlag>(static_cast<underlying>(lhs) & static_cast<underlying>(rhs));
}
constexpr UpdateFlag operator^(UpdateFlag lhs, UpdateFlag rhs) noexcept {
    using underlying = std::underlying_type_t<UpdateFlag>;
    return static_cast<UpdateFlag>(static_cast<underlying>(lhs) ^ static_cast<underlying>(rhs));
}
constexpr UpdateFlag operator~(UpdateFlag flag) noexcept {
    using underlying = std::underlying_type_t<UpdateFlag>;
    return static_cast<UpdateFlag>(~static_cast<underlying>(flag));
}
constexpr UpdateFlag& operator|=(UpdateFlag& lhs, UpdateFlag rhs) noexcept {
    return lhs = lhs | rhs;
}
constexpr UpdateFlag& operator&=(UpdateFlag& lhs, UpdateFlag rhs) noexcept {
    return lhs = lhs & rhs;
}
constexpr UpdateFlag& operator^=(UpdateFlag& lhs, UpdateFlag rhs) noexcept {
    return lhs = lhs ^ rhs;
}
template <typename Enum>
constexpr bool has_flag(Enum flags, Enum flag) noexcept {
    using underlying = std::underlying_type_t<Enum>;
    return (static_cast<underlying>(flags) & static_cast<underlying>(flag)) != 0;
}


}  // end namespace chrono

#endif
