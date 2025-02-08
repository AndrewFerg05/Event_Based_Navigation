/*
Filename    : Software/include/Logging.hpp
Author      : Samuel Kliskey
Project     : Event Based Navigation
Date        : 8/2/25
Description : Header file for logging messages
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
11-JAN-2025 SARK created to log errors and messages instead of in terminal
--------------------------------------------------------------------------------
*/

#ifndef LOGGING_HPP
#define LOGGING_HPP

//==============================================================================
// External Files
//------------------------------------------------------------------------------




//==============================================================================
//      Classes
//------------------------------------------------------------------------------



//==============================================================================
//      Function Prototypes
//------------------------------------------------------------------------------
void message(std::string source, std::string message);

void error(std::string source, std::string message);

#endif  // LOGGING_HPP
//==============================================================================
// End of File :  Software/include/Communication.hpp