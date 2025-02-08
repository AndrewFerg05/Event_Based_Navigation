/*
Filename    : Software/include/TypeAliases.hpp
Author      : Andrew Ferguson
Project     : Event Based Navigation
Date        : 14/1/25
Description : 
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
11-JAN-2025 AF created
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

// Function to initialize Google Logging
inline void initLogging(char* argv0) {

    google::InitGoogleLogging(argv0);


    // Specifies the directory where logs should be stored.
    FLAGS_log_dir = "./logs"; 

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

}



#endif  // TYPE_ALIASES_HPP
//==============================================================================
// End of File :  Software/include/TYPE_ALIASES_HPP.hpp