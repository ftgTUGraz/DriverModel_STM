#include <cmath>
#include "ManualDrivingBreaking.h"

int DrivingBrake::indexVelocityRange(double currentVelocity)
{
    int index = 0;
    DrivingBrake lookupTable;

    for (int i = 0; i < 8; i++)
    {
        if ((lookupTable.lookupTableVelocity[i][1] < currentVelocity) &&
            (currentVelocity <= lookupTable.lookupTableVelocity[i][0]))
        {
            index = i;
            break;
        }
    }
    return index;
}

int DrivingBrake::indexDecelVelocity(double targetVelocity)
{
    int index = 0;
    DrivingBrake lookupTable;

    for (int i = 0; i < 5; i++)
    {
        if ((lookupTable.lookupTableTgtVelocity[i][1] < targetVelocity) &&
            (targetVelocity <= lookupTable.lookupTableTgtVelocity[i][0]))
        {
            index = i;
            break;
        }
    }
    return index;
}

double DrivingBrake::meanDecelRate(int indexVelocity, int indexDecelV)
{
    DrivingBrake lookupTable;
    double Deceleration = 0.0;

    switch (indexVelocity)
    {
    case 0:
        Deceleration = lookupTable.lookupDeceleration1[indexDecelV];
        break;
    case 1:
        Deceleration = lookupTable.lookupDeceleration2[indexDecelV];
        break;
    case 2:
        Deceleration = lookupTable.lookupDeceleration3[indexDecelV];
        break;
    case 3:
        Deceleration = lookupTable.lookupDeceleration4[indexDecelV];
        break;
    case 4:
        Deceleration = lookupTable.lookupDeceleration4[indexDecelV];
        break;
    }
    return Deceleration;
}

double DrivingBrake::accelerationProfiles(double t, double vi, double vf, double td, double Ld, double am)
{
    double roh, A0, A1, A2, m, r, ram, a;
    vi *= 3.6;

    if (td == 0 || (vi - vf) == 0)
    {
        return 0;
    }

    roh = ((3.6 * Ld / td) - vf) / (vi - vf); // a ratio which relates to the shape of the speed - timecurve of acceleration
    A0 = 27 * roh - 19;
    A1 = A0 + 4;
    A2 = 6 * roh + 4;
    if (A2 == 0)
    {
        return 0;
    }
    if ((pow(A1, 2) - 4 * A0 * A2) < 0)
    {
        return 0;
    }
    m = (-A1 + pow((pow(A1, 2) - 4 * A0 * A2), 0.5)) / (2 * A2); // model calibration parameter
    if (m == 0)
    {
        return 0;
    }
    r = pow((1 + 2 * m), (2 + 1 / m)) / (4 * pow(m, 2)); // model parameter
    ram = r * am;
    a = (ram) / td * pow((1 - pow((t / td), m)), 2) * t; // calculate acceleration a profile curve with argument t

    return a;
}

double DrivingBrake::brakingTime(int decelInit)
{
    DrivingBrake lookupTable;
    return lookupTable.lookupTableBrkTime[decelInit];
}


double DrivingBrake::ACCDccelerationProfiles(double t, double decelInit, double decelTarget, double allowedDecelGrad)
{
    double diffDecel = abs(decelTarget - decelInit);
    //double modDecel;
    double x = 0;
    double a, b, c, decel;
    int j = 1;

    if (diffDecel > allowedDecelGrad)
    {
        for (int i = 1; i < 100; i++)
        {
            double res = i * allowedDecelGrad;

            if (res == diffDecel)
            {
                x = i;
                break;
            }

            else if (res < diffDecel)
            {
                j = i; // restore current i
            }

            else
            {
                x = (diffDecel - j * allowedDecelGrad) / allowedDecelGrad + j;
                break;
            }
        }
    }

    else // diffDecel < allowedDecelGrad
    {
        x = diffDecel / allowedDecelGrad;
    }


    // find the coeffiecent of polynomial equation
    c = decelInit;
    if (x == 0)
    {
        return 0;
    }
    a = (decelInit - decelTarget) / pow(x, 2);
    b = -2 * a * x;

    if (t <= (2 * x))
    {
        decel = a * pow(t, 2) + b * t + c;
    }

    else
    {
        decel = 0;
    }


    return decel;
}