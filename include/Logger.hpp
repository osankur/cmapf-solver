# pragma once

#include <boost/log/sources/severity_logger.hpp>
#include <boost/log/attributes.hpp>
#include <boost/log/core.hpp>
#include <boost/log/expressions.hpp>
#include <boost/log/utility/setup/file.hpp>
#include <boost/log/sources/severity_feature.hpp>
#include <boost/log/sources/record_ostream.hpp>

enum SeverityLevel
{
    TRACE,
    DEBUG,
    INFO,
    WARNING,
    ERROR,
    FATAL
};

BOOST_LOG_ATTRIBUTE_KEYWORD(severity, "Severity", SeverityLevel)

class Logger {
private:
    boost::log::sources::severity_logger<SeverityLevel> m_logger;

    Logger() : m_logger() {};

public:
    Logger(const Logger&)               = delete;
    Logger& operator=(const Logger&)    = delete;
    Logger(Logger&&)                    = delete;
    Logger& operator=(Logger&&)         = delete;

    static Logger& getSingleton() {
        static Logger logger;
        return logger;
    }

    static void init(const std::string& file, SeverityLevel level) {
        if (!file.empty())
            boost::log::add_file_log(file);
        boost::log::core::get()->set_filter(severity >= level);
    }

    static boost::log::sources::severity_logger<SeverityLevel>& getLogger() {
        return getSingleton().m_logger;
    }
};

#define LOG_TRACE(s)    BOOST_LOG_SEV(Logger::getLogger(), SeverityLevel::TRACE)    << s
#define LOG_DEBUG(s)    BOOST_LOG_SEV(Logger::getLogger(), SeverityLevel::DEBUG)    << s
#define LOG_INFO(s)     BOOST_LOG_SEV(Logger::getLogger(), SeverityLevel::INFO)     << s
#define LOG_WARNING(s)  BOOST_LOG_SEV(Logger::getLogger(), SeverityLevel::WARNING)  << s
#define LOG_ERROR(s)    BOOST_LOG_SEV(Logger::getLogger(), SeverityLevel::ERROR)    << s
#define LOG_FATAL(s)    BOOST_LOG_SEV(Logger::getLogger(), SeverityLevel::FATAL)    << s