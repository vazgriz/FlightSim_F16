using System.Collections;
using System.Collections.Generic;
using NUnit.Framework;
using UnityEngine;
using UnityEngine.TestTools;

public class ModelTestCase {
    // Textbook provides a table of input values and the expected output
    // Index    Param       Input           State           Output (delta)
    //     1    0.4 (XCG)   0.9 (throttle)  500 (vt)        -75.23724
    //     2                 20 (elevator)   0.5 (alpha)    -0.8813419
    //     3                -15 (aileron)   -0.2 (beta)     -0.4759990
    //     4                -20 (rudder)    -1 (phi)        
    //     5                                 1 (theta)      
    //     6                                -1 (psi)        
    //     7                                 0.7 (P)        12.62679
    //     8                                -0.8 (Q)        0.9649671
    //     9                                 0.9 (R)        0.5809759
    //    10                                1000 (north)    
    //    11                                 900 (east)     
    //    12                                10000 (alt)     248.1241
    //    13                                90 (power)      -58.68999

    const float weight = 25000;
    const float gd = 32.17f;
    const float mass = weight / gd;
    Vector4 inertiaTensor = new Vector4(9496, 55814, 63100, 982);

    // params
    const float XCG = 0.4f;

    // inputs
    const float throttle = 0.9f;
    const float elevator = 20;
    const float aileron = -15;
    const float rudder = -20;

    // state
    const float vt = 500;
    const float alpha = 0.5f;
    const float beta = -0.2f;
    const float P =  0.7f;
    const float Q = -0.8f;
    const float R =  0.9f;
    const float altitude = 10000;
    const float power = 90;

    const float vtAcceleration = -75.23724f;
    const float pDelta = 12.62679f;
    const float qDelta = 0.9649671f;
    const float rDelta = 0.5809759f;
    const float powerDelta = -58.68999f;

    [Test]
    public void ModelAerodynamicsTest() {
        const float dt = 1 / 60.0f;

        AirDataComputer adc = new AirDataComputer();
        AirData airData = adc.CalculateAirData(vt, altitude);

        Engine engine = new Engine();
        engine.Enabled = true;
        engine.ThrottleCommand = throttle;
        engine.PowerOutput = 90f;
        engine.Altitude = altitude;
        engine.Mach = airData.altitudeMach;

        engine.Update(dt);

        float thrust = engine.Thrust;

        AerodynamicState state = new AerodynamicState();
        state.airData = airData;
        state.inertiaTensor = inertiaTensor;
        state.velocity = new Vector3(0, 0, vt);
        state.alpha = alpha * Mathf.Rad2Deg;
        state.beta = beta * Mathf.Rad2Deg;
        // phi, theta, psi are not inputs
        state.angularVelocity = new Vector3(P, Q, R);
        // position north and east are not inputs
        // throttle is not an input
        state.controlSurfaces = new ControlSurfaces(elevator, rudder, aileron);
        state.xcg = XCG;

        Aerodynamics aero = new Aerodynamics();
        AerodynamicForces forces = aero.CalculateAerodynamics(state);

        // original text uses acceleration, need to calculate forces
        float vtForce = vtAcceleration * mass;
        //Assert.AreEqual(vtForce, forces.force.x + thrust, 0.1f);

        Assert.AreEqual(pDelta, forces.angularAcceleration.x, 0.1f);
        Assert.AreEqual(qDelta, forces.angularAcceleration.y, 0.1f);
        Assert.AreEqual(rDelta, forces.angularAcceleration.z, 0.1f);
    }

    [Test]
    public void ModelEngineTest() {
        float targetPower = Engine.CalculateThrottleGear(throttle);
        float pdot = Engine.CalculatePowerRateOfChange(power, targetPower);

        Assert.AreEqual(powerDelta, pdot, 0.1f);
    }

    [Test]
    public void ModelTable1DLookup() {
        float[] table = new float[12] {
            1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12
        };

        for (int i = 0; i < 12; i++) {
            float alpha = -10 + (i * 5);
            float value = Table.LinearLookup(alpha, 0.2f, table, -2, 9);
            Assert.AreEqual(table[i], value, 0.01f);
        }

        {
            float value = Table.LinearLookup(27.5f, 0.2f, table, -2, 9);
            Assert.AreEqual((table[7] + table[8]) * 0.5f, value, 0.01f);
        }

        {
            float value = Table.LinearLookup(50, 0.2f, table, -2, 9);
            Assert.AreEqual(Mathf.LerpUnclamped(table[10], table[11], 2), value, 0.01f);
        }

        {
            float value = Table.LinearLookup(-15, 0.2f, table, -2, 9);
            Assert.AreEqual(Mathf.LerpUnclamped(table[1], table[0], 2), value, 0.01f);
        }
    }

    [Test]
    public void ModelTable2DLookup() {
        float[,] table = new float[5, 12] {
             { 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12 },
             { 21, 22, 23, 24, 25, 26, 27, 28, 29, 30, 31, 32 },
             { 31, 32, 33, 34, 35, 36, 37, 38, 39, 40, 41, 42 },
             { 41, 42, 43, 44, 45, 46, 47, 48, 49, 50, 51, 52 },
             { 51, 52, 53, 54, 55, 56, 57, 58, 59, 60, 61, 62 }
        };

        for (int i = 0; i < 12; i++) {
            for (int j = 0; j < 5; j++) {
                float alpha = -10 + (i * 5);
                float beta = -10 + (j * 5);
                float value = Table.BilinearLookup(alpha, 0.2f, beta, 0.2f, table, -2, 9, -2, 2);
                Assert.AreEqual(table[j, i], value, 0.01f);
            }
        }

        {
            float alpha = 27.5f;
            float beta = 7.5f;
            float value = Table.BilinearLookup(alpha, 0.2f, beta, 0.2f, table, -2, 9, -2, 2);

            float v0 = Mathf.LerpUnclamped(table[3, 7], table[3, 8], 0.5f);
            float v1 = Mathf.LerpUnclamped(table[4, 7], table[4, 8], 0.5f);
            Assert.AreEqual(Mathf.LerpUnclamped(v0, v1, 0.5f), value, 0.01f);
        }

        {
            float alpha = 50;
            float beta = 7.5f;
            float value = Table.BilinearLookup(alpha, 0.2f, beta, 0.2f, table, -2, 9, -2, 2);

            float v0 = Mathf.LerpUnclamped(table[3, 10], table[3, 11], 2);
            float v1 = Mathf.LerpUnclamped(table[4, 10], table[4, 11], 2);
            Assert.AreEqual(Mathf.LerpUnclamped(v0, v1, 0.5f), value, 0.01f);
        }

        {
            float alpha = 27.5f;
            float beta = 15f;
            float value = Table.BilinearLookup(alpha, 0.2f, beta, 0.2f, table, -2, 9, -2, 2);

            float v0 = Mathf.LerpUnclamped(table[3, 7], table[3, 8], 0.5f);
            float v1 = Mathf.LerpUnclamped(table[4, 7], table[4, 8], 0.5f);
            Assert.AreEqual(Mathf.LerpUnclamped(v0, v1, 2), value, 0.01f);
        }
    }
}
