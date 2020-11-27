#pragma once

/* parameters for driver braking model*/
//#define VI 71.0 // vi -> initial speed km / h
#define VF 5.5 // vf -> final speed km / h
#define TD 17.2 // td -> acceleration / deceleration time(ta or td) s
#define LD 216.6 // Ld -> deceleration distance m
#define AM -1.71 // am -> maximum deceleration rate

/* parameters for AEB braking model*/
#define aeb_vi 70.0 // vi -> initial speed km / h
#define aeb_vf 0 // vf -> final speed km / h
#define aeb_td 4 // td -> acceleration / deceleration time(ta or td) s
#define aeb_Ld 56 // Ld -> deceleration distance m
#define aeb_am -10 // am -> maximum deceleration rate

/* parameters for ACC braking model*/
#define DECELINIT 0.0 // initial decleration at aprroaching phase m / s2
#define DECEL_TARGET -3.0 // target deceleration of ACC arriving m / m2
#define ALLOWED_DECEL_GRAD 1.5 // allowed deceleration gradient based on different approaching velocity m / s3

/* parameters for lane change model*/
#define TM 5 // average lane change manoeuvre time
#define H 3.5 // width of the highway trajectory
#define accelLaneChg 1.2 // acceleration peak value during lane change
#define laneChgAngDeviation 0.01 // Determine whether the current angle reported to midlle line is greater than the threshold of turing angle deviation
#define laneChgDirect -1 // lane change direction: left turn -1 / right turn 1
#define meanVelocity 85 // define mean velocity of lane change
#define triggerTime 1// TTC trigger time for lane change
#define testingVehID 1 // set the testing vehicle's ID number

/* Math parameters*/
#define Pi 3.14159265358979323846 // circumference

#define TTC1 1 // define ttc time is 1s
#define TTC2 2 // define ttc time is 2s
#define TTC3 3 // define ttc time is 3s