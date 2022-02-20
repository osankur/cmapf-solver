/* Copyright (c) 2021 Arthur Queffelec
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 */
#pragma once

#include <string>

#include <boost/log/attributes.hpp>
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/sources/record_ostream.hpp>
#include <boost/log/sources/severity_feature.hpp>
#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/utility/setup/file.hpp>

enum SeverityLevel { kTrace, kDebug, kInfo, kWarning, kError, kFatal };

BOOST_LOG_ATTRIBUTE_KEYWORD(severity, "Severity", SeverityLevel)

class Logger {
 private:
  boost::log::sources::severity_logger<SeverityLevel> logger_;

  Logger() : logger_() {}

 public:
  Logger(const Logger&) = delete;
  Logger& operator=(const Logger&) = delete;
  Logger(Logger&&) = delete;
  Logger& operator=(Logger&&) = delete;

  static Logger& get_singleton() {
    static Logger logger;
    return logger;
  }

  static void Init(const std::string& file, SeverityLevel level) {
    if (!file.empty()) boost::log::add_file_log(file);
    boost::log::core::get()->set_filter(severity >= level);
  }

  static boost::log::sources::severity_logger<SeverityLevel>& get_logger() { return get_singleton().logger_; }
};
/*
#define LOG_TRACE(s) BOOST_LOG_SEV(Logger::get_logger(), SeverityLevel::kTrace) << s
#define LOG_DEBUG(s) BOOST_LOG_SEV(Logger::get_logger(), SeverityLevel::kDebug) << s
#define LOG_INFO(s) BOOST_LOG_SEV(Logger::get_logger(), SeverityLevel::kInfo) << s
#define LOG_WARNING(s) BOOST_LOG_SEV(Logger::get_logger(), SeverityLevel::kWarning) << s
#define LOG_ERROR(s) BOOST_LOG_SEV(Logger::get_logger(), SeverityLevel::kError) << s
#define LOG_FATAL(s) BOOST_LOG_SEV(Logger::get_logger(), SeverityLevel::kFatal) << s
*/
#define LOG_TRACE(s) std::cerr << "TRACE: " << s << "\n"
#define LOG_DEBUG(s) std::cerr << "DEBUG: " << s << "\n"
#define LOG_INFO(s) std::cerr << "INFO: " << s << "\n"
#define LOG_WARNING(s) std::cerr << "WARNING: " << s << "\n"
#define LOG_ERROR(s) std::cerr << "ERROR: " << s << "\n"
#define LOG_FATAL(s) std::cerr << "FATAL: " << s << "\n"
