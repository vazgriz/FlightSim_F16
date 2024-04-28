using System;
using System.Collections.Generic;
using UnityEngine;

public class SimpleTrimmer {
    AirDataComputer airDataComputer;
    Aerodynamics aerodynamics;

    float mass;
    Vector3 momentOfInertia;

    public struct SimulatedState {
        public Vector3 velocity;
        public Vector3 acceleration;
        public float altitude;
        public float alpha;
        public float pitchRate;
        public float pitch;
    }

    public SimpleTrimmer(AirDataComputer airDataComputer, Aerodynamics aerodynamics, float mass, Vector3 momentOfInertia) {
        this.airDataComputer = airDataComputer;
        this.aerodynamics = aerodynamics;

        this.mass = mass;
        this.momentOfInertia = momentOfInertia;
    }

    public SimulatedState Trim(float dt, float timeMax, float airspeed, float altitude, float initialAlpha, float elevator, float gravity) {
        Vector3 velocity = new Vector3(airspeed, 0, 0);
        SimulatedState state = new SimulatedState() {
            velocity = velocity,
            altitude = altitude,
            alpha = initialAlpha,
            pitchRate = 0,
            pitch = initialAlpha
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
                airData = airData,
                alpha = state.alpha,
                altitude = state.altitude,
                velocity = localVelocity,
                controlSurfaces = new Vector3(elevator, 0, 0)
            };

            var aeroForces = aerodynamics.CalculateAerodynamics(aeroState);

            // altitude is invariant

            // integrate in local space
            state.acceleration = (aeroForces.force / mass) + new Vector3(0, 0, gravity);
            state.velocity += state.acceleration * dt;

            float angularAcceleration = aeroForces.moment.y / momentOfInertia.y;
            state.pitchRate += angularAcceleration * dt;
            state.pitch += state.pitchRate * dt;

            Quaternion newRotation = Quaternion.Euler(0, state.pitch, 0);
            Vector3 worldVelocity = Quaternion.Inverse(newRotation) * state.velocity;
            worldVelocity.y = 0;
            Vector3 worldVelNormalized = worldVelocity.normalized;

            // assume airspeed magnitude does not change (no drag, no thrust)
            Vector3 newVelocity = worldVelNormalized * airspeed;
            state.velocity = newRotation * newVelocity;
            Vector3 velNormalized = state.velocity.normalized;

            state.alpha = Mathf.Atan2(velNormalized.z, velNormalized.x) * Mathf.Rad2Deg;
            state.pitch = 0;

            time += dt;
        }

        return state;
    }
}