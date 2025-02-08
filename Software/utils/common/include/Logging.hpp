/*
Filename    : Software/include/Logging.hpp
Author      : Andrew Ferguson
Project     : Event Based Navigation
Date        : 8/2/25
Description : 
--------------------------------------------------------------------------------
*/

#ifndef LOGGING_HPP
#define LOGGING_HPP

//==============================================================================
// External Files
//------------------------------------------------------------------------------
#include <cstdint>
#pragma diagnostic push
#pragma GCC diagnostic ignored "-Wunused-local-typedefs"
// glog has an unused typedef.
// https://github.com/google/glog/pull/33
#include <glog/logging.h>
#pragma diagnostic pop

#include <glog/logging.h>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <fstream>

//! @file logging.hpp
//! Includes Glog framework and defines macros for DEBUG_CHECK_* which
//! can be compiled away.

//==============================================================================
//  Macros
//------------------------------------------------------------------------------
#define DEBUG_CHECK(val) CHECK(val)
#define DEBUG_CHECK_NOTNULL(val) CHECK_NOTNULL(val)
#define DEBUG_CHECK_EQ(val1, val2) CHECK_EQ(val1, val2)
#define DEBUG_CHECK_NE(val1, val2) CHECK_NE(val1, val2)
#define DEBUG_CHECK_LE(val1, val2) CHECK_LE(val1, val2)
#define DEBUG_CHECK_LT(val1, val2) CHECK_LT(val1, val2)
#define DEBUG_CHECK_GE(val1, val2) CHECK_GE(val1, val2)
#define DEBUG_CHECK_GT(val1, val2) CHECK_GT(val1, val2)
#define DEBUG_CHECK_DOUBLE_EQ(val1, val2) CHECK_DOUBLE_EQ(val1, val2)
#define DEBUG_CHECK_NEAR(val1, val2, margin) CHECK_NEAR(val1, val2, margin)
//==============================================================================
//      Function Prototypes
//------------------------------------------------------------------------------
class AlignedLogSink : public google::LogSink {
    public:
        std::ofstream logFile;
    
        AlignedLogSink() {
            logFile.open("../logs/full_log.txt", std::ios::app); // Change path if needed
            if (logFile.is_open()) {
                std::time_t now = std::time(nullptr);
                logFile << "======================================PROGRAM=START=" << std::put_time(std::localtime(&now), "%Y-%m-%d %H:%M:%S") << "======================================" << std::endl;
            }
        }
    
        ~AlignedLogSink() {
            if (logFile.is_open()) {
                logFile.close();
            }
        }
    
        void send(google::LogSeverity severity, const char* full_filename, 
                  const char* base_filename, int line, 
                  const struct ::tm* tm_time, const char* message, size_t message_len) override {
            std::ostringstream formatted_log;
            formatted_log << google::GetLogSeverityName(severity)[0]
                          << google::GetLogSeverityName(severity)[1]
                          << google::GetLogSeverityName(severity)[2]
                          << " "
                          << std::put_time(tm_time, "%Y%m%d %H:%M:%S")  // Timestamp
                          << " " << std::setw(20) << std::left << base_filename  // Pad filenames to 20 chars
                          << "Line:" << std::setw(4) << line  // Pad line numbers to 4 digits
                          << " " << std::string(message, message_len) << std::endl;
    
            std::cerr << formatted_log.str();  // Send to stderr
    
            if (logFile.is_open()) {
                logFile << formatted_log.str();  // Write to custom log file
            }
        }
    };

// Function to initialize Google Logging
inline void initLogging(char* argv0, AlignedLogSink* sink) {

    google::InitGoogleLogging(argv0);

    // Specifies the directory where logs should be stored.
    FLAGS_log_dir = "../logs"; 

    FLAGS_log_prefix = false;  // Disable default log format

    //    0 - logs only go to log files
    //    1 - logs to both log files and console
    FLAGS_alsologtostderr = 1;


    //    0 = INFO    (Logs everything: INFO, WARNING, ERROR, FATAL)
    //    1 = WARNING (Logs WARNING, ERROR, FATAL)
    //    2 = ERROR   (Logs ERROR, FATAL)
    //    3 = FATAL   (Logs FATAL)
    FLAGS_minloglevel = 0;

    //    0 - logs are written to files
    //    1 - logs are printed to stderr
    FLAGS_logtostderr = 0;

    google::AddLogSink(sink);  // Register custom log sink
}

inline void endLogging(AlignedLogSink* sink){
    google::RemoveLogSink(sink);
}



#endif  // LOGGING_HPP
//==============================================================================
// End of File :  Software/include/Logging.hpp