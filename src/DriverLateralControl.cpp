#include <cmath>
#include "DriverLateralControl.h"
#include "BrakingBehaviorParameters.h"

double LaneChange::TimeManoeuvreCalc(double velocity, double frictionCoeff)
{
	return (frictionCoeff * (8 + 0.5 * velocity) + 5) / (10 * frictionCoeff);
}


double LaneChange::AccelLaneChange(double accelPeak, double tManoeuvre, double t)
{
	return (accelPeak*sin(2 * Pi / tManoeuvre * t));
}


double LaneChange::LaneChangeAngle(double t, double tManoeuvre, double newPosX, double newPosY, double oldPosX, double oldPosY)
{
	double slope;

	if (t <= tManoeuvre)
	{
		slope = atan((newPosY - oldPosY) / (newPosX - oldPosX));
	}
	else
	{
		slope = 0;
	}

	return slope;
}

double LaneChange::fctPosX(double t, double tManoeuvre, double meanV)
{
	if (t <= tManoeuvre)
	{
		return (meanV / 3.6 * t);
	}
	else
	{
		return 0;
	}
}

double LaneChange::fctPosY(double t, double tManoeuvre, double width)
{
	if (t <= tManoeuvre)
	{
		return abs((-6 * width / pow(tManoeuvre, 5)) * pow(t, 5) + (15 * width / pow(tManoeuvre, 4)) * pow(t, 4) + (-10 * width / pow(tManoeuvre, 3)) * pow(t, 3));
	}
	else
	{
		return 0;
	}
}