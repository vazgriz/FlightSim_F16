using System;
using System.Collections.Generic;
using UnityEngine;

public class Engine {
    public const float militaryPowerThrottle = 0.77f;

    float[,] idlePowerTable;
    float[,] militaryPowerTable;
    float[,] maxPowerTable;

    float targetPower;
    float currentPower;

    public bool Enabled { get; set; }

    /// <summary>
    /// Commanded throttle of the engine in normalized percent [0, 1]
    /// </summary>
    public float ThrottleCommand { get; set; }

    /// <summary>
    /// Power command of the engine in normalized percent [0, 1]
    /// </summary>
    public float PowerCommand {
        get {
            return targetPower;
        }
    }

    /// <summary>
    /// Power output by the engine in percent [0, 100]
    /// </summary>
    public float PowerOutput {
        get {
            return currentPower;
        }
        set {
            currentPower = value;
        }
    }

    /// <summary>
    /// Altitude in feet
    /// </summary>
    public float Altitude { get; set; }

    /// <summary>
    /// Speed in Mach
    /// </summary>
    public float Mach { get; set; }

    /// <summary>
    /// Thrust output by the engine
    /// </summary>
    public float Thrust { get; private set; }

    public Engine() {
        idlePowerTable = new float[6, 6] {
            {  1060.0f,   670.0f,   880.0f, 1140.0f, 1500.0f, 1860.0f },
            {   635.0f,   425.0f,   690.0f, 1010.0f, 1330.0f, 1700.0f },
            {    60.0f,    25.0f,   345.0f,  755.0f, 1130.0f, 1525.0f },
            { -1020.0f,  -710.0f,  -300.0f,  350.0f,  910.0f, 1360.0f },
            { -2700.0f, -1900.0f, -1300.0f, -247.0f,  600.0f, 1100.0f },
            { -3600.0f, -1400.0f,  -595.0f, -342.0f, -200.0f,  700.0f },
        };

        militaryPowerTable = new float[6, 6] {
            { 12680.0f,  9150.0f, 6200.0f, 3950.0f, 2450.0f, 1400.0f },
            { 12680.0f,  9150.0f, 6313.0f, 4040.0f, 2470.0f, 1400.0f },
            { 12610.0f,  9312.0f, 6610.0f, 4290.0f, 2600.0f, 1560.0f },
            { 12640.0f,  9839.0f, 7090.0f, 4660.0f, 2840.0f, 1660.0f },
            { 12390.0f, 10176.0f, 7750.0f, 5320.0f, 3250.0f, 1930.0f },
            { 11680.0f,  9848.0f, 8050.0f, 6100.0f, 3800.0f, 2310.0f },
        };

        maxPowerTable = new float[6, 6] {
            { 20000.0f, 15000.0f, 10800.0f,  7000.0f, 4000.0f, 2500.0f },
            { 21420.0f, 15700.0f, 11225.0f,  7323.0f, 4435.0f, 2600.0f },
            { 22700.0f, 16860.0f, 12250.0f,  8154.0f, 5000.0f, 2835.0f },
            { 24240.0f, 18910.0f, 13760.0f,  9285.0f, 5700.0f, 3215.0f },
            { 26070.0f, 21075.0f, 15975.0f, 11115.0f, 6860.0f, 3950.0f },
            { 28886.0f, 23319.0f, 18300.0f, 13484.0f, 8642.0f, 5057.0f },
        };
    }

    public void Update(float dt) {
        if (!Enabled) {
            currentPower = 0;
            Thrust = 0;
            return;
        }

        targetPower = CalculateThrottleGear(ThrottleCommand);
        float powerChangeRate = CalculatePowerRateOfChange(currentPower, targetPower);
        currentPower = Utilities.MoveTo(currentPower, targetPower, Mathf.Abs(powerChangeRate), dt, 0, 100);

        Thrust = CalculateThrust(currentPower, Altitude, Mach);
    }

    public static float CalculateThrottleGear(float throttle) {
        // maps throttle 0 - 0.77   to power 0% - 50%
        // maps throttle 0.77 - 1.0 to power 50% - 100%

        float power;

        if (throttle <= militaryPowerThrottle) {
            power = 64.94f * throttle;
        } else {
            power = 217.38f * throttle - 117.38f;
        }

        return power;
    }

    public static float InvertThrottleGear(float power) {
        float throttle;

        if (power <= 50.0f) {
            throttle = power / 64.94f;
        } else {
            throttle = (power + 117.38f) / 217.38f;
        }

        return throttle;
    }

    public static float CalculatePowerRateOfChange(float actualPower, float commandPower) {
        // calculates how fast power output should change based on commanded power
        float T;
        float p2;

        if (commandPower >= 50.0) {
            if (actualPower >= 50.0) {
                T = 5.0f;
                p2 = commandPower;
            } else {
                p2 = 60.0f;
                T = CalculateRTau(p2 - actualPower);
            }
        } else {
            if (actualPower >= 50.0) {
                T = 5.0f;
                p2 = 40.0f;
            } else {
                p2 = commandPower;
                T = CalculateRTau(p2 - actualPower);
            }
        }

        float pdot = T * (p2 - actualPower);

        return pdot;
    }

    static float CalculateRTau(float dp) {
        float rTau;

        if (dp <= 25.0) {
            rTau = 1.0f;
        } else if (dp >= 50.0) {
            rTau = 0.1f;
        } else {
            rTau = 1.9f - 0.036f * dp;
        }

        return rTau;
    }

    float InterpolateThrust(float thrust1, float thrust2, float power) {
        float result = Mathf.LerpUnclamped(thrust1, thrust2, power * 0.02f);
        return result;
    }

    float CalculateThrust(float power, float altitude, float rMach) {
        float a = Mathf.Max(0, altitude);
        float m = Mathf.Max(0, rMach);

        float thrust;
        float thrustMilitary = Table.BilinearLookup(a, 0.0001f, m, 5, militaryPowerTable, 0, 5, 0, 5);

        // perform trilinear interpolation
        if (power < 50.0) {
            float thrustIdle = Table.BilinearLookup(a, 0.0001f, m, 5, idlePowerTable, 0, 5, 0, 5);
            thrust = InterpolateThrust(thrustIdle, thrustMilitary, power);
        } else {
            float thrustMax = Table.BilinearLookup(a, 0.0001f, m, 5, maxPowerTable, 0, 5, 0, 5);
            thrust = InterpolateThrust(thrustMilitary, thrustMax, power - 50.0f);
        }

        return thrust;
    }
}
