using System;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.UIElements;

public struct AerodynamicState {
    public Vector4 inertiaTensor;
    public Vector3 velocity;
    public Vector3 angularVelocity;
    public AirData airData;
    public float altitude;
    public float alpha;
    public float beta;
    public float xcg;
    public Vector3 controlSurfaces;
}

public struct AerodynamicForces {
    public Vector3 force;
    public Vector3 angularAcceleration;
}

public class Aerodynamics {
    const float wingAreaFtSquared = 300;
    const float wingSpanFt = 30;
    const float CBAR = 11.32f;
    const float XCGR = 0.35f;
    const float HX = 160;

    float[,] dampTable;
    float[,] xAxisTable;
    float[] zAxisTable;

    float[] dampingTable;
    float[,] xMomentTable;
    float[,] yMomentTable;
    float[,] zMomentTable;
    float[,] dldaTable;
    float[,] dldrTable;
    float[,] dndaTable;
    float[,] dndrTable;

    public Aerodynamics() {
        dampTable = new float[9, 12] {
            { -0.267f, -0.110f,  0.308f,  1.340f,  2.080f,  2.910f,  2.760f,  2.050f,  1.500f,  1.490f,  1.830f,  1.210f },
            {  0.882f,  0.852f,  0.876f,  0.958f,  0.962f,  0.974f,  0.819f,  0.483f,  0.590f,  1.210f, -0.493f, -1.040f },
            { -0.108f, -0.108f, -0.188f,  0.110f,  0.258f,  0.226f,  0.344f,  0.362f,  0.611f,  0.529f,  0.298f, -2.270f },
            { -8.800f, -25.80f, -28.90f, -31.40f, -31.20f, -30.70f, -27.70f, -28.20f, -29.00f, -29.80f, -38.30f, -35.30f },
            { -0.126f, -0.026f,  0.063f,  0.113f,  0.208f,  0.230f,  0.319f,  0.437f,  0.680f,  0.100f,  0.447f, -0.330f },
            { -0.360f, -0.359f, -0.443f, -0.420f, -0.383f, -0.375f, -0.329f, -0.294f, -0.230f, -0.210f, -0.120f, -0.100f },
            { -7.210f, -5.400f, -5.230f, -5.260f, -6.110f, -6.640f, -5.690f, -6.000f, -6.200f, -6.400f, -6.600f, -6.000f },
            { -0.380f, -0.363f, -0.378f, -0.386f, -0.370f, -0.453f, -0.550f, -0.582f, -0.595f, -0.637f, -1.020f, -0.840f },
            {  0.061f,  0.052f,  0.052f, -0.012f, -0.013f, -0.024f,  0.050f,  0.150f,  0.130f,  0.158f,  0.240f,  0.150f }
        };

        xAxisTable = new float[5, 12] {
            { -0.099f, -0.081f, -0.081f, -0.063f, -0.025f, 0.044f, 0.097f, 0.113f, 0.145f, 0.167f, 0.174f, 0.166f },
            { -0.048f, -0.038f, -0.040f, -0.021f,  0.016f, 0.083f, 0.127f, 0.137f, 0.162f, 0.177f, 0.179f, 0.167f },
            { -0.022f, -0.020f, -0.021f, -0.004f,  0.032f, 0.094f, 0.128f, 0.130f, 0.154f, 0.161f, 0.155f, 0.138f },
            { -0.040f, -0.038f, -0.039f, -0.025f,  0.006f, 0.062f, 0.087f, 0.085f, 0.100f, 0.110f, 0.104f, 0.091f },
            { -0.083f, -0.073f, -0.076f, -0.072f, -0.046f, 0.012f, 0.024f, 0.025f, 0.043f, 0.053f, 0.047f, 0.040f },
        };

        zAxisTable = new float[12] {
            0.770f, 0.241f, -0.100f, -0.416f, -0.731f, -1.053f, -1.366f, -1.646f, -1.917f, -2.120f, -2.248f, -2.229f
        };

        dampingTable = new float[9];

        xMomentTable = new float[7, 12] {
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
            { -0.001f, -0.004f, -0.008f, -0.012f, -0.016f, -0.019f, -0.020f, -0.020f, -0.015f, -0.008f, -0.013f, -0.015f },
            { -0.003f, -0.009f, -0.017f, -0.024f, -0.030f, -0.034f, -0.040f, -0.037f, -0.016f, -0.002f, -0.010f, -0.019f },
            { -0.001f, -0.010f, -0.020f, -0.030f, -0.039f, -0.044f, -0.050f, -0.049f, -0.023f, -0.006f, -0.014f, -0.027f },
            {  0.000f, -0.010f, -0.022f, -0.034f, -0.047f, -0.046f, -0.059f, -0.061f, -0.033f, -0.036f, -0.035f, -0.035f },
            {  0.007f, -0.010f, -0.023f, -0.034f, -0.049f, -0.046f, -0.068f, -0.071f, -0.060f, -0.058f, -0.062f, -0.059f },
            {  0.009f, -0.011f, -0.023f, -0.037f, -0.050f, -0.047f, -0.074f, -0.079f, -0.091f, -0.076f, -0.077f, -0.076f },
        };

        yMomentTable = new float[5, 12] {
            {  0.205f,  0.168f,  0.186f,  0.196f,  0.213f,  0.251f,  0.245f,  0.238f,  0.252f,  0.231f,  0.198f,  0.192f },
            {  0.081f,  0.077f,  0.107f,  0.110f,  0.110f,  0.141f,  0.127f,  0.119f,  0.133f,  0.108f,  0.081f,  0.093f },
            { -0.046f, -0.020f, -0.009f, -0.005f, -0.006f,  0.010f,  0.006f, -0.001f,  0.014f,  0.000f, -0.013f,  0.032f },
            { -0.174f, -0.145f, -0.121f, -0.127f, -0.129f, -0.102f, -0.097f, -0.113f, -0.087f, -0.084f, -0.069f, -0.006f },
            { -0.259f, -0.202f, -0.184f, -0.193f, -0.199f, -0.150f, -0.160f, -0.167f, -0.104f, -0.076f, -0.041f, -0.005f }
        };

        zMomentTable = new float[7, 12] {
            { 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0 },
            {  0.018f,  0.019f,  0.018f,  0.019f,  0.019f,  0.018f,  0.013f,  0.007f,  0.004f, -0.014f, -0.017f, -0.033f },
            {  0.038f,  0.042f,  0.042f,  0.042f,  0.043f,  0.039f,  0.030f,  0.017f,  0.004f, -0.035f, -0.047f, -0.057f },
            {  0.056f,  0.057f,  0.059f,  0.058f,  0.058f,  0.053f,  0.032f,  0.012f,  0.002f, -0.046f, -0.071f, -0.073f },
            {  0.064f,  0.077f,  0.076f,  0.074f,  0.073f,  0.057f,  0.029f,  0.007f,  0.012f, -0.034f, -0.065f, -0.041f },
            {  0.074f,  0.086f,  0.093f,  0.089f,  0.080f,  0.062f,  0.049f,  0.022f,  0.028f, -0.012f, -0.002f, -0.013f },
            {  0.079f,  0.090f,  0.106f,  0.106f,  0.096f,  0.080f,  0.068f,  0.030f,  0.064f,  0.015f,  0.011f, -0.001f },
        };

        dldaTable = new float[7, 12] {
            { -0.041f, -0.052f, -0.053f, -0.056f, -0.050f, -0.056f, -0.082f, -0.059f, -0.042f, -0.038f, -0.027f, -0.017f },
            { -0.041f, -0.053f, -0.053f, -0.053f, -0.050f, -0.051f, -0.066f, -0.043f, -0.038f, -0.027f, -0.023f, -0.016f },
            { -0.042f, -0.053f, -0.052f, -0.051f, -0.049f, -0.049f, -0.043f, -0.035f, -0.026f, -0.016f, -0.018f, -0.014f },
            { -0.040f, -0.052f, -0.051f, -0.052f, -0.048f, -0.048f, -0.042f, -0.037f, -0.031f, -0.026f, -0.017f, -0.012f },
            { -0.043f, -0.049f, -0.048f, -0.049f, -0.043f, -0.042f, -0.042f, -0.036f, -0.025f, -0.021f, -0.016f, -0.011f },
            { -0.044f, -0.048f, -0.048f, -0.047f, -0.042f, -0.041f, -0.020f, -0.028f, -0.013f, -0.014f, -0.011f, -0.010f },
            { -0.043f, -0.049f, -0.047f, -0.045f, -0.042f, -0.037f, -0.003f, -0.013f, -0.010f, -0.003f, -0.007f, -0.008f },
        };

        dldrTable = new float[7, 12] {
            {  0.005f,  0.017f,  0.014f,  0.010f, -0.005f,  0.009f,  0.019f,  0.005f,  0.000f, -0.005f, -0.011f,  0.008f },
            {  0.007f,  0.016f,  0.014f,  0.014f,  0.013f,  0.009f,  0.012f,  0.005f,  0.000f,  0.004f,  0.009f,  0.007f },
            {  0.013f,  0.013f,  0.011f,  0.012f,  0.011f,  0.009f,  0.008f,  0.005f, -0.002f,  0.005f,  0.003f,  0.005f },
            {  0.018f,  0.015f,  0.015f,  0.014f,  0.014f,  0.014f,  0.014f,  0.015f,  0.013f,  0.011f,  0.006f,  0.001f },
            {  0.015f,  0.014f,  0.013f,  0.013f,  0.012f,  0.011f,  0.011f,  0.010f,  0.008f,  0.008f,  0.007f,  0.003f },
            {  0.021f,  0.011f,  0.010f,  0.011f,  0.010f,  0.009f,  0.008f,  0.010f,  0.006f,  0.005f,  0.000f,  0.001f },
            {  0.023f,  0.010f,  0.011f,  0.011f,  0.011f,  0.010f,  0.008f,  0.010f,  0.006f,  0.014f,  0.020f,  0.000f },
        };

        dndaTable = new float[7, 12] {
            {  0.001f, +0.027f, -0.017f, -0.013f, -0.012f, -0.016f,  0.001f,  0.017f,  0.011f,  0.017f,  0.008f,  0.016f },
            {  0.002f, -0.014f, -0.016f, -0.016f, -0.014f, -0.019f, -0.021f,  0.002f,  0.012f,  0.015f,  0.015f,  0.011f },
            { -0.006f, -0.008f, -0.006f, -0.006f, -0.005f, -0.008f, -0.005f,  0.007f,  0.004f,  0.007f,  0.006f,  0.006f },
            { -0.011f, -0.011f, -0.010f, -0.009f, -0.008f, -0.006f,  0.000f,  0.004f,  0.007f,  0.010f,  0.004f,  0.010f },
            { -0.015f, -0.015f, -0.014f, -0.012f, -0.011f, -0.008f, -0.002f,  0.002f,  0.006f,  0.012f,  0.011f,  0.011f },
            { -0.024f, -0.010f, -0.004f, -0.002f, -0.001f,  0.003f,  0.014f,  0.006f, -0.001f,  0.004f,  0.004f,  0.006f },
            { -0.022f,  0.002f, -0.003f, -0.005f, -0.003f, -0.001f, -0.009f, -0.009f, -0.001f,  0.003f, -0.002f,  0.001f },
        };

        dndrTable = new float[7, 12] {
            { -0.018f, -0.052f, -0.052f, -0.052f, -0.054f, -0.049f, -0.059f, -0.051f, -0.030f, -0.037f, -0.026f, -0.013f },
            { -0.028f, -0.051f, -0.043f, -0.046f, -0.045f, -0.049f, -0.057f, -0.052f, -0.030f, -0.033f, -0.030f, -0.008f },
            { -0.037f, -0.041f, -0.038f, -0.040f, -0.040f, -0.038f, -0.037f, -0.030f, -0.027f, -0.024f, -0.019f, -0.013f },
            { -0.048f, -0.045f, -0.045f, -0.045f, -0.044f, -0.045f, -0.047f, -0.048f, -0.049f, -0.045f, -0.033f, -0.016f },
            { -0.043f, -0.044f, -0.041f, -0.041f, -0.040f, -0.038f, -0.034f, -0.035f, -0.035f, -0.029f, -0.022f, -0.009f },
            { -0.052f, -0.034f, -0.036f, -0.036f, -0.035f, -0.028f, -0.024f, -0.023f, -0.020f, -0.016f, -0.010f, -0.014f },
            { -0.062f, -0.034f, -0.027f, -0.028f, -0.027f, -0.027f, -0.023f, -0.023f, -0.019f, -0.009f, -0.025f, -0.010f },
        };
    }

    float ReadDampTable(float[,] table, int x, int y) {
        const int xStart = -2;
        return table[y, x - xStart];
    }

    void CalculateDampingValues(float alpha) {
        // Calculate damping array
        //  D[0] = CXq
        //  D[1] = CYr
        //  D[2] = CYp
        //  D[3] = CZq
        //  D[4] = Clr
        //  D[5] = Clp
        //  D[6] = Cmq
        //  D[7] = Cnr
        //  D[8] = Cnp

        float S = 0.2f * alpha;
        int K = Mathf.Clamp((int)S, -1, 8);

        float DA = S - K;
        int L = K + (int)Mathf.Sign(DA);

        for (int i = 0; i < 9; i++) {
            dampingTable[i] = ReadDampTable(dampTable, K, i) + Math.Abs(DA) * (ReadDampTable(dampTable, L, i) - ReadDampTable(dampTable, K, i));
        }
    }

    public float EstimateGForceX(AirData airData, float alpha, float elevator) {
        float QS = airData.qBar * wingAreaFtSquared;
        return QS * GetZAxisForce(alpha, 0, elevator);
    }

    public AerodynamicForces CalculateAerodynamics(AerodynamicState currentState) {
        AerodynamicForces result = new AerodynamicForces();

        Vector3 forceCoefficient = GetForceCoefficient(
            currentState.alpha, currentState.beta,
            currentState.controlSurfaces.z, currentState.controlSurfaces.y, currentState.controlSurfaces.x
        );

        Vector3 momentCoefficient = GetMomentCoefficient(
            currentState.alpha, currentState.beta, currentState.controlSurfaces.x
        );

        // calculate inertia values
        float AXX = currentState.inertiaTensor.x;
        float AYY = currentState.inertiaTensor.y;
        float AZZ = currentState.inertiaTensor.z;
        float AXZ = currentState.inertiaTensor.w;

        float AXZS = AXZ * AXZ;
        float XPQ = AXZ * (AXX - AYY + AZZ);
        float GAM = AXX * AZZ - AXZS;
        float XQR = AZZ * (AZZ - AYY) + AXZS;
        float ZPQ = (AZZ - AYY) * AXX + AXZS;
        float YPR = AZZ - AXX;

        CalculateDampingValues(currentState.alpha);

        // P Q R
        float P = currentState.angularVelocity.x;   // roll rate
        float Q = currentState.angularVelocity.y;   // pitch rate
        float R = currentState.angularVelocity.z;   // yaw rate

        float airspeed = Mathf.Max(1, currentState.velocity.magnitude);
        float TVT = 0.5f / airspeed;

        float B2V = wingSpanFt * TVT;
        float CQ = CBAR * Q * TVT;

        float DAIL = currentState.controlSurfaces.z / 20.0f;
        float DRDR = currentState.controlSurfaces.y / 30.0f;

        // damping
        float CXT = forceCoefficient.x + CQ * dampingTable[0];
        float CYT = forceCoefficient.y + B2V * (dampingTable[1] * R + dampingTable[2] * P);
        float CZT = forceCoefficient.z + CQ * dampingTable[3];

        float CLT = momentCoefficient.x + B2V * (dampingTable[4] * R + dampingTable[5] * P);
        CLT += GetDLDA(currentState.alpha, currentState.beta) * DAIL;
        CLT += GetDLDR(currentState.alpha, currentState.beta) * DRDR;
        float CMT = momentCoefficient.y + CQ * dampingTable[6] + CZT * (XCGR - currentState.xcg);
        float CNT = momentCoefficient.z + B2V * (dampingTable[7] * R + dampingTable[8] * P) - CYT * (XCGR - currentState.xcg) * CBAR / wingSpanFt;
        CNT += GetDNDA(currentState.alpha, currentState.beta) * DAIL;
        CNT += GetDNDR(currentState.alpha, currentState.beta) * DRDR;

        float QS = currentState.airData.qBar * wingAreaFtSquared;
        float QSB = QS * wingSpanFt;

        // forces
        // Acceleration in original text. Need to calculate force instead of acceleration
        float UDOT = QS * CXT;
        float VDOT = QS * CYT;
        float WDOT = QS * CZT;

        float ROLL = QSB * CLT;
        float PITCH = QS * CBAR * CMT;
        float YAW = QSB * CNT;
        float PQ = P * Q;
        float QR = Q * R;
        float QHX = Q * HX;

        float rollAccel = ((XPQ * PQ) - (XQR * QR) + (AZZ * ROLL) + (AXZ * (YAW + QHX))) / GAM;
        float pitchAccel = ((YPR * P * R) - (AXZ * (P * P - R * R)) + PITCH - (R * HX)) / AYY;
        float yawAccel = ((ZPQ * PQ) - (XPQ * QR) + (AXZ * ROLL) + (AXX * (YAW + QHX))) / GAM;

        result.force = new Vector3(UDOT, VDOT, WDOT);
        result.angularAcceleration = new Vector3(rollAccel, pitchAccel, yawAccel);

        return result;
    }

    Vector3 GetForceCoefficient(float alpha, float beta, float aileron, float rudder, float elevator) {
        return new Vector3(
            GetXAxisForce(alpha, elevator),
            GetYAxisForce(beta, aileron, rudder),
            GetZAxisForce(alpha, beta, elevator)
        );
    }

    Vector3 GetMomentCoefficient(float alpha, float beta, float elevator) {
        return new Vector3(
            GetXAxisMoment(alpha, beta),
            GetYAxisMoment(alpha, elevator),
            GetZAxisMoment(alpha, beta)
        );
    }

    float GetXAxisForce(float alpha, float elevator) {
        float result = Table.BilinearLookup(alpha, 0.2f, elevator, 1f / 12f, xAxisTable, -2, 9, -2, 2);
        return result;
    }

    float GetYAxisForce(float beta, float aileron, float rudder) {
        float CY = -0.02f * beta + 0.021f * (aileron / 20.0f) + 0.086f * (rudder / 30.0f);
        return CY;
    }

    float GetZAxisForce(float alpha, float beta, float elevator) {
        float S = Table.LinearLookup(alpha, 0.2f, zAxisTable, -2, 10);
        float CZ = S * (1 - Mathf.Pow(beta * Mathf.Deg2Rad, 2)) - 0.19f * (elevator / 25.0f);
        return CZ;
    }

    float GetYAxisMoment(float alpha, float elevator) {
        float result = Table.BilinearLookup(alpha, 0.2f, elevator, 1f / 12f, yMomentTable, -2, 9, -2, 2);
        return result;
    }

    float GetXAxisMoment(float alpha, float beta) {
        float DUM = Table.BilinearLookup(alpha, 0.2f, Mathf.Abs(beta), 0.2f, xMomentTable, -2, 9, 0, 7);
        float CL = DUM * Mathf.Sign(beta);

        return CL;
    }

    float GetZAxisMoment(float alpha, float beta) {
        float DUM = Table.BilinearLookup(alpha, 0.2f, Mathf.Abs(beta), 0.2f, zMomentTable, -2, 9, 0, 7);
        float CL = DUM * Mathf.Sign(beta);

        return CL;
    }

    float GetDLDA(float alpha, float beta) {
        float dlda = Table.BilinearLookup(alpha, 0.2f, beta, 0.1f, dldaTable, -2, 9, -3, 3);
        return dlda;
    }

    float GetDLDR(float alpha, float beta) {
        float dldr = Table.BilinearLookup(alpha, 0.2f, beta, 0.1f, dldrTable, -2, 9, -3, 3);
        return dldr;
    }

    float GetDNDA(float alpha, float beta) {
        float dnda = Table.BilinearLookup(alpha, 0.2f, beta, 0.1f, dndaTable, -2, 9, -3, 3);
        return dnda;
    }

    float GetDNDR(float alpha, float beta) {
        float dndr = Table.BilinearLookup(alpha, 0.2f, beta, 0.1f, dndrTable, -2, 9, -3, 3);
        return dndr;
    }
}
