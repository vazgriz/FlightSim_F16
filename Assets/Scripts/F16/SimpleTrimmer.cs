using System;
using System.Collections.Generic;
using UnityEngine;

public class SimpleTrimmer {
    AirDataComputer airDataComputer;
    Aerodynamics aerodynamics;

    float mass;
    Vector4 momentOfInertia;

    public struct SimulatedState {
        public Vector3 velocity;
        public Vector3 acceleration;
        public float altitude;
        public float alpha;
        public float pitchRate;
        public float pitch;
    }

    public SimpleTrimmer(AirDataComputer airDataComputer, Aerodynamics aerodynamics, float mass, Vector4 momentOfInertia) {
        this.airDataComputer = airDataComputer;
        this.aerodynamics = aerodynamics;

        this.mass = mass;
        this.momentOfInertia = momentOfInertia;
    }

    public SimulatedState Trim(float dt, float timeMax, Vector3 initialSpeed, float altitude, float initialAlpha, float pitchRate, float elevator, float gravity) {
        Vector3 velocity = new Vector3(initialSpeed.x, 0, initialSpeed.z);
        float airspeed = velocity.magnitude;

        SimulatedState state = new SimulatedState() {
            velocity = velocity,
            altitude = altitude,
            alpha = initialAlpha,
            pitchRate = pitchRate,
            pitch = 0
        };

        if (dt <= 0 || timeMax <= 0) {
            return state;
        }

        float time = 0;

        while (time < timeMax) {
            Quaternion rotation = Quaternion.Euler(0, state.pitch, 0);
            Vector3 localVelocity = rotation * velocity;

            AirData airData = airDataComputer.CalculateAirData(airspeed, altitude);
            AerodynamicState aeroState = new AerodynamicState() {
                inertiaTensor = momentOfInertia,
                airData = airData,
                alpha = state.alpha,
                altitude = state.altitude,
                velocity = localVelocity,
                controlSurfaces = new Vector3(elevator, 0, 0)
            };

            var aeroForces = aerodynamics.CalculateAerodynamics(aeroState);

            // altitude is invariant
            // pitch is always 0
            // local space always matches world space at the start of a loop

            // integrate in local space
            state.acceleration = aeroForces.force / mass;
            state.velocity += state.acceleration * dt;

            float angularVelocityY = aeroForces.angularVelocity.y;
            state.pitchRate = angularVelocityY;
            float pitchDelta = angularVelocityY * dt;

            // rotate velocity by pitchDelta
            Quaternion newRotation = Quaternion.Euler(0, -pitchDelta, 0);
            Vector3 newVelocity = newRotation * state.velocity;
            newVelocity.y = 0;
            newVelocity.z += gravity * dt;
            Vector3 velNormalized = newVelocity.normalized;

            // assume airspeed magnitude does not change (no drag, no thrust)
            state.velocity = velNormalized * airspeed;

            state.alpha = Mathf.Atan2(velNormalized.z, velNormalized.x) * Mathf.Rad2Deg;

            time += dt;
        }

        return state;
    }
}