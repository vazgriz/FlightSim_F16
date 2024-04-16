using System;
using System.Collections.Generic;
using UnityEngine;

public struct AirData {
    public float altitudeMach;
    public float qBar;
}

public class AirDataComputer {
    /// <summary>
    /// Density in slugs/ft^3
    /// </summary>
    public const float SeaLevelDensity = 2.377e-3f;
    public const float MaxAltitude = 35000.0f;

    /// <summary>
    /// Calculates air data based on velocity and altitude
    /// </summary>
    /// <param name="velocity">Velocity in ft/s</param>
    /// <param name="altitude">Altitude in ft</param>
    /// <returns>Air data</returns>
    public AirData CalculateAirData(float velocity, float altitude) {
        altitude = Mathf.Clamp(altitude, 0, MaxAltitude);
        float temperatureFactor = 1.0f - 0.703e-5f * altitude;
        float T = 519.0f * temperatureFactor;   // temperature in Rankine

        if (altitude >= MaxAltitude) {
            T = 390.0f;
        }

        float rho = SeaLevelDensity * Mathf.Pow(temperatureFactor, 4.14f);
        float altitudeMach = velocity / Mathf.Sqrt(1.4f * 1716.3f * T);
        float qBar = 0.5f * rho * velocity * velocity;

        return new AirData() {
            altitudeMach = altitudeMach,
            qBar = qBar
        };
    }
}
