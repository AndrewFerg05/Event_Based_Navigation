/*
Filename    : Software/src/Logging.cpp
Author      : Samuel Kliskey
Project     : Event Based Navigation
Date        : 10/1/25
Description : Logging in project
--------------------------------------------------------------------------------
*/

//==============================================================================
// External Files
//------------------------------------------------------------------------------
// External
#include <iostream>
#include <fstream>
#include <ctime>
#include <mutex>
#include <iomanip>

// Local
#include  "Logging.hpp"



//==============================================================================
// Function Prototypes
//------------------------------------------------------------------------------

//==============================================================================
// MACROs
//------------------------------------------------------------------------------
#define LOG_FILE   "../messages.txt"
#define ERROR_FILE "../errors.txt"

//==============================================================================
// Global Variable Initialisation
//------------------------------------------------------------------------------
std::mutex logMutex;
std::mutex errMutex;

//==============================================================================
// Functions
//------------------------------------------------------------------------------
void message(std::string source, std::string message){
    std::lock_guard<std::mutex> lock(logMutex);
    std::ofstream log(LOG_FILE, std::ios::app); // Open log file in append mode
    if (log.is_open()) {
        // Get current time
        std::time_t now = std::time(nullptr);
        std::tm localTime = *std::localtime(&now); // Convert to local time
        
        log << std::left << std::setw(6) << ("(" + source + ")") << " at ["
        << std::put_time(&localTime, "%Y-%m-%d %H:%M:%S") // Properly formatted timestamp
        << "] : " << message << std::endl;
        log.close();
    } else {
        std::cerr << "Error: Unable to open log file!" << std::endl;
    }
}

void error(std::string source, std::string message){
    std::lock_guard<std::mutex> lock(errMutex);
    std::ofstream log(ERROR_FILE, std::ios::app); // Open log file in append mode
    if (log.is_open()) {
        // Get current time
        std::time_t now = std::time(nullptr);
        std::tm localTime = *std::localtime(&now); // Convert to local time
        
        log << std::left << "ERROR " << std::setw(6) << ("(" + source + ")") << " at ["
        << std::put_time(&localTime, "%Y-%m-%d %H:%M:%S") // Properly formatted timestamp
        << "] : " << message << std::endl;
        log.close();
    } else {
        std::cerr << "Error: Unable to open log file!" << std::endl;
    }
}
//==============================================================================
// End of File : Software/src/Logging.cpp