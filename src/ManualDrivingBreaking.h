#pragma once

#include "Utilities.h"
#include "BreakingBehaviorParameters.h"

class DrivingBrake {

public:
    // input velocity range between 40kph - 90kph
    double lookupTableVelocity[5][2] = {
        {90.0 , 80.0},
        {80.0 , 70.0},
        {70.0 , 60.0},
        {60.0 , 50.0},
        {50.0 , 40.0}
    };

    // input target velocity range between 0kph - 80kph 
    double lookupTableTgtVelocity[8][2] = {
        {10.0, 0.0},
        {20.0, 10.0},
        {30.0, 20.0},
        {40.0, 30.0},
        {50.0, 40.0},
        {60.0, 50.0},
        {70.0, 60.0},
        {90.0, 80.0}
    };

    // input lookup table of braking time 
    double lookupTableBrkTime[5] =
    { 17.2, 16.8, 13.4, 12.6, 10.1 };

    // lookup table for mean deceleration rate based on different target velocity
    double lookupDeceleration1[9] =
    { -0.87, -1.9, -2.07, -2.12, -2.02, -1.83, -1.34, -0.91, -0.48 };

    double lookupDeceleration2[9] =
    { -0.89, -2.0, -1.71, -1.83, -1.76, -1.37, -0.78, -0.45, 0.0 };

    double lookupDeceleration3[9] =
    { -0.88, -1.91, -2.12, -2.06, -1.75, -1.07, -0.58, 0.0, 0.0 };

    double lookupDeceleration4[9] =
    { -0.84, -1.87, -1.92, -1.67, -1.1, -0.58, 0.0, 0.0, 0.0 };

    double lookupDeceleration5[9] =
    { -0.91, -1.92, -1.82, -1.26, -0.67, 0, 0, 0, 0 };

    // find the correspond index of current velocity
    int indexVelocityRange(double currentVelocity);

    // find the correspond index of deceleration velocity
    int indexDecelVelocity(double targetVelocity);

    // find the correspond index of deceleration velocity
    double meanDecelRate(int indexVelocity, int indexDecelV);


    /*
     ** calculate Driver deceleration profile **

        vi -> initial speed km / h
        vf -> final speed km / h
        td -> acceleration / deceleration time(ta or td) s
        Ld -> deceleration distance m
        am -> maximum deceleration rate
        t  -> argument time t s
    */
    double accelerationProfiles(double t, double vi, double vf = VF, double td = TD, double Ld = LD, double am = AM);

    /* determine braking duration time*/
    double brakingTime(int decelInit);

    /*
     ** calculate ACC mode deceleration profile **

               decelInit -> initial decleration at aprroaching phase m / s2
             decelTarget -> target deceleration of ACC arriving m / m2
        allowedDecelGrad -> allowed deceleration gradient based on different approaching velocity m / s3
                       t -> argument time t s
    */
    double ACCDccelerationProfiles(double t, double decelInit, double decelTarget = DECEL_TARGET, double allowedDecelGrad = ALLOWED_DECEL_GRAD);


};