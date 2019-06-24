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

#include "hal/Device.hpp"

#include "hal/Error.hpp"
#include "hal/IBoard.hpp"

namespace hal {

Device::Device(SharingPolicy sharingPolicy, IBoard& board)
    : m_sharingPolicy(sharingPolicy)
    , m_board(board)
{}

std::error_code Device::take()
{
    if (m_sharingPolicy == SharingPolicy::eSingle && m_ownersCount == 1)
        return Error::eDeviceTaken;

    ++m_ownersCount;
    return Error::eOk;
}

std::error_code Device::give()
{
    if (m_ownersCount == 0)
        return Error::eDeviceNotTaken;

    --m_ownersCount;
    return Error::eOk;
}

std::error_code returnDevice(std::shared_ptr<Device>& device)
{
    if (!device)
        return Error::eInvalidArgument;

    if (device->ownersCount() == 0)
        return Error::eDeviceNotTaken;

    if (auto error = IBoard::GetBoard(device).returnDevice(device))
        return error;

    device.reset();
    return Error::eOk;
}

} // namespace hal
