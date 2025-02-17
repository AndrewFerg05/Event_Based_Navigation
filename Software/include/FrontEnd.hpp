/*
Filename    : Software/include/FrontEnd.hpp
Author      : Samuel Kliskey
Project     : Event Based Navigation
Date        : 11/1/25
Description : Header file for the FrontEnd (FE) thread
--------------------------------------------------------------------------------
Change History
--------------------------------------------------------------------------------
11-JAN-2025 SARK created to design code structure
--------------------------------------------------------------------------------
*/

#ifndef FRONTEND_HPP
#define FRONTEND_HPP

//==============================================================================
// External Files
//------------------------------------------------------------------------------
#include "ThreadInterface.hpp"
#include "TypeAliases.hpp"
#include <opencv2/opencv.hpp>
#include <iostream>
#include <cmath>

//==============================================================================
//      Classes
//------------------------------------------------------------------------------
class FrontEnd
{
    public:
    explicit FrontEnd(
        std::shared_ptr<CommunicationManager> comms);
    ~FrontEnd() = default;
    void start();
    void stop();
    void idle();
    void addData(
        const StampedImage& ,
        const StampedEventArray&,
        const ImuStamps&,
        const ImuAccGyrContainer&,
        const bool& no_motion_prior);
    void addImuData(
        int64_t stamp,
        const Vector3& acc, 
        const Vector3& gyr);

    private:
    std::shared_ptr<CommunicationManager> comms_interface_;
};

//==============================================================================
//      Function Prototypes
//------------------------------------------------------------------------------




#endif  // FRONTEND_HPP
//==============================================================================
// End of File :  Software/include/Communication.hpp