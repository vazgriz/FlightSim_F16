using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

[Serializable]
public class PIDController {
    public float P;
    public float I;
    public float D;

    public float min;
    public float max;

    public float Calculate(float dt, float value, float velocity, float target) {
        float result = 0;
        float error = target - value;
        result += error * P;
        result += -velocity * D;

        return Mathf.Clamp(result, min, max);
    }
}
