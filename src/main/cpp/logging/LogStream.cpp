// Copyright (c) 2014-2021 FRC Team 3512. All Rights Reserved.

#include "logging/LogStream.hpp"

#include "logging/LogStreambuf.hpp"

LogStream::LogStream(Logger& logger) : std::ostream(new LogStreambuf(logger)) {}

LogStream::~LogStream() { delete rdbuf(); }

void LogStream::SetLevel(LogEvent::VerbosityLevel level) {
    static_cast<LogStreambuf*>(rdbuf())->SetLevel(level);
}
