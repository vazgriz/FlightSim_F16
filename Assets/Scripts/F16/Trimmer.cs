using System;
using System.Collections.Generic;
using UnityEngine;

public class Trimmer {
    struct CostInput {
        public float throttle;
        public float elevator;
        public float aileron;
        public float rudder;
    }

    const int vSize = 32;
    const int ySize = 33;
    float[,] v;
    float[] y;

    public Trimmer() {
        v = new float[vSize, vSize];
        y = new float[ySize];
    }

    public void Trim(int degreesOfFreedomCount, Func<float> costFunc) {

    }

    void SetY(int index, float value) {
        const int offset = 1;
        y[index - offset] = value;
    }

    void Simplex(Func<float[], float> fx, int n, float[] X, float[] DX, float sd, float m, float y0, float YL) {
        int nv = n + 1;

        for (int i = 0; i < n; i++) {
            for (int j = 0; j < nv; j++) {
                v[j, i] = X[i];
                v[i + 1, i] = X[i] + DX[i];
            }
        }

        y0 = fx(X);

        for (int j = 1; j < nv; j++) {
            //SetY(j, fx(v[j, 0]));
        }
    }

    //float CostF16(float[] s) {
    //
    //}
}