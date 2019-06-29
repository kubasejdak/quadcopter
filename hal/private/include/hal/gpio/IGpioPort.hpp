/////////////////////////////////////////////////////////////////////////////////////
///
/// @file
/// @author Kuba Sejdak
/// @copyright BSD 2-Clause License
///
/// Copyright (c) 2019, Kuba Sejdak <kuba.sejdak@gmail.com>
/// All rights reserved.
///
/// Redistribution and use in source and binary forms, with or without
/// modification, are permitted provided that the following conditions are met:
///
/// 1. Redistributions of source code must retain the above copyright notice, this
///    list of conditions and the following disclaimer.
///
/// 2. Redistributions in binary form must reproduce the above copyright notice,
///    this list of conditions and the following disclaimer in the documentation
///    and/or other materials provided with the distribution.
///
/// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
/// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
/// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
/// DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
/// FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
/// DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
/// SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
/// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
/// OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
/// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
///
/////////////////////////////////////////////////////////////////////////////////////

#pragma once

#include "hal/Error.hpp"
#include "hal/gpio/types.hpp"

#include <system_error>
#include <type_traits>

namespace hal::gpio {

/// @enum Access
/// Represents the GPIO access type.
enum class Access { eReadOnly, eWriteOnly, eReadWrite };

namespace detail {
// clang-format off

/// @typedef IsReadOnly
/// @tparam access          Demanded GPIO access type.
/// Metafunction that checks, if the demanded type is Access::eReadOnly.
template <Access access>
using IsReadOnly = std::is_same<std::integral_constant<Access, access>, std::integral_constant<Access, Access::eReadOnly>>;

/// @typedef IsReadOnly
/// @tparam access          Demanded GPIO access type.
/// Metafunction that checks, if the demanded type is Access::eWriteOnly.
template <Access access>
using IsWriteOnly = std::is_same<std::integral_constant<Access, access>, std::integral_constant<Access, Access::eWriteOnly>>;

/// @typedef IsReadOnly
/// @tparam access          Demanded GPIO access type.
/// Metafunction that checks, if the demanded type is Access::eReadWrite.
template <Access access>
using IsReadWrite = std::is_same<std::integral_constant<Access, access>, std::integral_constant<Access, Access::eReadWrite>>;

/// @typedef IfReadable
/// Metafunction used to conditionally add/remove "read" capability to the GPIO class via SFIANE expression.
/// @tparam access          Demanded GPIO access type.
template<Access access>
using IfReadable = std::enable_if_t<std::disjunction<IsReadOnly<access>, IsReadWrite<access>>::value, int>;

/// @typedef IfWriteable
/// Metafunction used to conditionally add/remove "write" capability to the GPIO class via SFIANE expression.
/// @tparam AccessType      Demanded GPIO class access type.
template<Access access>
using IfWriteable = std::enable_if_t<std::disjunction<IsWriteOnly<access>, IsReadWrite<access>>::value, int>;

// clang-format on
} // namespace detail

/// @class IGpioPort
/// @tparam WidthType       Type representing the bit-width of the port (e.g. std::uint32_t means that port is 32-bit).
/// @tparam access          Demanded access type. This parameter will enable/disable the read/write capabilities.
/// Represents the GPIO port with the defined width and access type.
template <typename WidthType, Access access>
class IGpioPort {
    static_assert(isValidWidthType<WidthType>, "IGpioPort can be parametrized only with unsigned arithmetic types");

public:
    /// Virtual destructor.
    virtual ~IGpioPort() = default;

    /// Reads the demanded set of GPIO port bits defined by the mask.
    /// @tparam accessType  Demanded access type.
    /// @param data         Output argument where the read value will be stored.
    /// @param mask         Mask defining which port bits should be read.
    /// @return Error code of the operation.
    template <Access accessType = access, typename = detail::IfReadable<accessType>>
    std::error_code read(WidthType& data, WidthType mask)
    {
        return drvRead(data, mask);
    }

    /// Writes the demanded set of GPIO port bits defined by the mask.
    /// @tparam accessType  Demanded access type.
    /// @param value        Value to be written to the GPIO port.
    /// @param mask         Mask defining which port bits should be written.
    /// @return Error code of the operation.
    template <Access accessType = access, typename = detail::IfWriteable<accessType>>
    std::error_code write(WidthType value, WidthType mask)
    {
        return drvWrite(value, mask);
    }

private:
    /// Driver specific implementation of GPIO port reading.
    /// @return Error code of the operation.
    virtual std::error_code drvRead(WidthType&, WidthType) { return Error::eNotSupported; }

    /// Driver specific implementation of GPIO port writing.
    /// @return Error code of the operation.
    virtual std::error_code drvWrite(WidthType, WidthType) { return Error::eNotSupported; }
};

} // namespace hal::gpio
