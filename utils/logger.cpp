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

#include "utils/logger.hpp"

#include "utils/Error.hpp"

#include <spdlog/sinks/rotating_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>
#include <spdlog/spdlog.h>

#include <memory>
#include <string_view>

namespace utils {

constexpr std::string_view cDefaultLogFormat("[%d.%m.%Y %T.%e] [%t] [%l] %v");

std::error_code initLogger()
{
    // Console logger: only warnings, errors and criticals.
    auto console = std::make_shared<spdlog::sinks::stdout_color_sink_mt>();
    if (!console)
        return Error::eConsoleLoggerFailure;

    console->set_level(spdlog::level::warn);
    console->set_pattern(cDefaultLogFormat.data());

    // Rotating file logger: all logs, 3x 5MB files.
    constexpr std::size_t cLogFileSize = 5 * 1024 * 1024;
    constexpr std::size_t cLogFilesCount = 3;
    auto file = std::make_shared<spdlog::sinks::rotating_file_sink_mt>("logs/quadcopter", cLogFileSize, cLogFilesCount);
    if (!file)
        return Error::eFileLoggerFailure;

    file->set_level(spdlog::level::trace);
    file->set_pattern(cDefaultLogFormat.data());

    auto logger = std::make_shared<spdlog::logger>("multiSink", spdlog::sinks_init_list({console, file}));
    if (!logger)
        return Error::eDefaultLoggerFailure;

    spdlog::set_default_logger(logger);
    return Error::eOk;
}

} // namespace utils
