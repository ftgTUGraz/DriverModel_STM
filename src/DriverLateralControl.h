#pragma once
#include "Utilities.h"

class LaneChange {

public:

    //double Tavg = 4.3; // time average to lane change
    
    /************************************************************
    * calculate minimum manoeuvre time:                         *
    * velocity -> current velocity m / s                        *
    * frictionCoeff -> maximum tyre-road friction coefficient   *
    ************************************************************/
    double TimeManoeuvreCalc(double velocity, double frictionCoeff);

    /*********************************************************************
    * calculate lane change trajectory:                                  *
    * t -> time step reprted to Time manoeuvre                           *
    * tManoeuvre -> which can be calculate by TimeManoeuvreCacl function *
    * width -> width of the highway trajectory                           *
    *********************************************************************/
    double LaneChangeAngle(double t, double tManoeuvre, double newPosX, double newPosY, double oldPosX, double oldPosY);

    /*********************************************************************
    * update position:                                                   *
    * t -> time step reprted to Time manoeuvre                           *
    * tManoeuvre -> which can be calculate by TimeManoeuvreCacl function *
    * width -> width of the highway trajectory                           *       
    * meanV -> mean velocity of lane change                              *
    *********************************************************************/
    double fctPosX(double t, double tManoeuvre, double meanV); // update position X
    double fctPosY(double t, double tManoeuvre, double width); // update position Y

    /*********************************************************************
    * calculate lane change acceleration:                                *
    * t -> time step reprted to Time manoeuvre                           *
    * tManoeuvre -> which can be calculate by TimeManoeuvreCacl function *
    * accelPeak -> max/min acceleation value                             *
    *********************************************************************/
    double AccelLaneChange(double accelPeak, double tManoeuvre, double t);
};