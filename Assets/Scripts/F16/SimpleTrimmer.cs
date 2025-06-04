using System;
using System.Collections.Generic;
using UnityEngine;

public class SimpleTrimmer {
    AirDataComputer airDataComputer;
    Aerodynamics aerodynamics;

    float mass;
    Vector4 momentOfInertia;
    float elevatorRange;
    float elevatorSpeed;

    public struct SimulatedState {
        public Vector3 velocity;
        public Vector3 acceleration;
        public float maxAccelerationZ;
        public float altitude;
        public float alpha;
        public float maxAlpha;
        public float pitchRate;
        public float pitch;
    }

    public SimpleTrimmer(AirDataComputer airDataComputer, Aerodynamics aerodynamics, float mass, Vector4 momentOfInertia, float elevatorRange, float elevatorSpeed) {
        this.airDataComputer = airDataComputer;
        this.aerodynamics = aerodynamics;

        this.mass = mass;
        this.momentOfInertia = momentOfInertia;
        this.elevatorRange = elevatorRange;
        this.elevatorSpeed = elevatorSpeed;
    }

    public SimulatedState Trim(float dt, float timeMax, SimulatedState initialState, float pitchRate, float gravity, PIDController pitchController, float xcg) {
        Vector3 velocity = new Vector3(initialState.velocity.x, 0, initialState.velocity.z);
        float airspeed = velocity.magnitude;
        float elevator = 0;

        SimulatedState state = new SimulatedState() {
            velocity = velocity,
            altitude = initialState.altitude,
            alpha = initialState.alpha,
            pitchRate = initialState.pitchRate,
            pitch = 0
        };

        if (dt <= 0 || timeMax <= 0) {
            return state;
        }

        float time = 0;

        while (time < timeMax) {
            float currentPitchRate = state.pitchRate * Mathf.Rad2Deg;
            float targetPitchRate = pitchRate * Mathf.Rad2Deg;
            float targetElevator = -pitchController.Calculate(dt, currentPitchRate, currentPitchRate, targetPitchRate);
            elevator = Utilities.MoveTo(elevator, targetElevator, elevatorSpeed, dt, -elevatorRange, elevatorRange);

            AirData airData = airDataComputer.CalculateAirData(airspeed, state.altitude);
            AerodynamicState aeroState = new AerodynamicState() {
                inertiaTensor = momentOfInertia,
                airData = airData,
                alpha = state.alpha,
                xcg = xcg,
                velocity = state.velocity,
                controlSurfaces = new ControlSurfaces(elevator, 0, 0),
                angularVelocity = new Vector3(0, state.pitchRate, 0)
            };

            var aeroForces = aerodynamics.CalculateAerodynamics(aeroState);

            // altitude is invariant
            // pitch is always 0
            // local space always matches world space at the start of a loop

            // integrate in local space
            state.acceleration = aeroForces.force / mass;
            state.velocity += state.acceleration * dt;

            float angularVelocityY = aeroForces.angularAcceleration.y;
            state.pitchRate = Mathf.Lerp(state.pitchRate, angularVelocityY, dt);
            float pitchDelta = state.pitchRate * Mathf.Rad2Deg * dt;

            if (Mathf.Abs(pitchDelta) > 360f) {
                break;
            }

            // rotate velocity by pitchDelta
            Quaternion newRotation = Quaternion.Euler(0, pitchDelta, 0);
            Vector3 newVelocity = newRotation * state.velocity;
            newVelocity.y = 0;
            newVelocity.z += gravity * dt;
            Vector3 velNormalized = newVelocity.normalized;

            // assume airspeed magnitude does not change (no drag, no thrust)
            state.velocity = velNormalized * airspeed;

            state.alpha = Mathf.Atan2(velNormalized.z, velNormalized.x) * Mathf.Rad2Deg;

            state.maxAlpha = Mathf.Max(state.maxAlpha, state.alpha);
            state.maxAccelerationZ = Mathf.Min(state.maxAccelerationZ, state.acceleration.z);

            time += dt;
        }

        return state;
    }
}