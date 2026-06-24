// =============================================================================
// PROJECT CHRONO - http://projectchrono.org
//
// Copyright (c) 2026 projectchrono.org
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

/// @addtogroup chrono_physics
/// @{

/// Flags that control the update operations. 
/// These flags can be combined with bitwise operators.
/// For example:
/// <pre>
///   UpdateFlags flags = UpdateFlags::DYNAMICS | UpdateFlags::JACOBIANS;
///   if (has_flag(flags, UpdateFlags::DYNAMICS)) { ... }
/// </pre>
enum class UpdateFlags : uint8_t {
    NONE = 0u,
    DYNAMICS = 1u << 0,
    JACOBIANS = 1u << 1,
    VISUAL_ASSETS = 1u << 2,
    UPDATE_ALL = DYNAMICS | JACOBIANS | VISUAL_ASSETS,
    UPDATE_ALL_NO_VISUAL = DYNAMICS | JACOBIANS
};

constexpr UpdateFlags operator|(UpdateFlags lhs, UpdateFlags rhs) noexcept {
    using underlying = std::underlying_type_t<UpdateFlags>;
    return static_cast<UpdateFlags>(static_cast<underlying>(lhs) | static_cast<underlying>(rhs));
}
constexpr UpdateFlags operator&(UpdateFlags lhs, UpdateFlags rhs) noexcept {
    using underlying = std::underlying_type_t<UpdateFlags>;
    return static_cast<UpdateFlags>(static_cast<underlying>(lhs) & static_cast<underlying>(rhs));
}
constexpr UpdateFlags operator^(UpdateFlags lhs, UpdateFlags rhs) noexcept {
    using underlying = std::underlying_type_t<UpdateFlags>;
    return static_cast<UpdateFlags>(static_cast<underlying>(lhs) ^ static_cast<underlying>(rhs));
}
constexpr UpdateFlags operator~(UpdateFlags flag) noexcept {
    using underlying = std::underlying_type_t<UpdateFlags>;
    return static_cast<UpdateFlags>(~static_cast<underlying>(flag));
}
constexpr UpdateFlags& operator|=(UpdateFlags& lhs, UpdateFlags rhs) noexcept {
    return lhs = lhs | rhs;
}
constexpr UpdateFlags& operator&=(UpdateFlags& lhs, UpdateFlags rhs) noexcept {
    return lhs = lhs & rhs;
}
constexpr UpdateFlags& operator^=(UpdateFlags& lhs, UpdateFlags rhs) noexcept {
    return lhs = lhs ^ rhs;
}
template <typename Enum>
constexpr bool has_flag(Enum flags, Enum flag) noexcept {
    using underlying = std::underlying_type_t<Enum>;
    return (static_cast<underlying>(flags) & static_cast<underlying>(flag)) != 0;
}

/// @} chrono_physics

}  // end namespace chrono

#endif
