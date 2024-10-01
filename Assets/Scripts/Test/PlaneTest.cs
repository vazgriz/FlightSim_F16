using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlaneTest : MonoBehaviour {
    [SerializeField]
    bool showForces;
    [SerializeField]
    bool showMoment;
    [SerializeField]
    float mass;
    [SerializeField]
    Vector3 inertiaTensor;
    [SerializeField]
    float altitude;
    [SerializeField]
    float speed;
    [SerializeField]
    bool runTrimmer;
    [SerializeField]
    float elevatorRange;
    [SerializeField]
    float elevatorSpeed;
    [SerializeField]
    float trimmerSimTimeStep;
    [SerializeField]
    float trimmerSimTime;
    [SerializeField]
    float trimmerPitchRate;
    [SerializeField]
    PIDController trimmerPID;

    Vector3 rollPitchYaw;
    float alpha;

    new Transform transform;
    AirDataComputer airDataComputer;
    Aerodynamics aerodynamics;
    Engine engine;

    SimpleTrimmer simpleTrimmer;

    void Start() {
        transform = GetComponent<Transform>();

        airDataComputer = new AirDataComputer();
        aerodynamics = new Aerodynamics();
        engine = new Engine();

        simpleTrimmer = new SimpleTrimmer(airDataComputer, aerodynamics, mass * Plane.kilosToPounds / (-Physics.gravity.y * Plane.metersToFeet), inertiaTensor, elevatorRange, elevatorSpeed);
    }

    void FixedUpdate() {
        UpdateForces();
        UpdateTrimmer();
    }

    void UpdateForces() {
        var euler = transform.eulerAngles;
        rollPitchYaw = new Vector3(
            Utilities.ConvertAngle360To180(euler.x),
            Utilities.ConvertAngle360To180(euler.y),
            Utilities.ConvertAngle360To180(euler.z)
        );

        alpha = rollPitchYaw.x;

        var airData = airDataComputer.CalculateAirData(speed, altitude);

        AerodynamicState state = new() {
            velocity = new Vector3(speed, 0, 0),
            angularVelocity = new Vector3(0, 0, 0),
            airData = airData,
            altitude = altitude,
            alpha = alpha,
            beta = rollPitchYaw.y,
            controlSurfaces = new Vector3(0, 0, 0)
        };

        var forces = aerodynamics.CalculateAerodynamics(state);
        var aeroForces = forces.force;
        var aeroMoment = forces.angularAcceleration;

        if (showForces) {
            float m = 1;

            if (mass != 0) {
                m = mass;
            }

            var f = aeroForces * Plane.poundsForceToNewtons / m;

            Debug.DrawLine(transform.position, transform.position + (transform.rotation * new Vector3(f.y, 0, 0)), Color.red);
            Debug.DrawLine(transform.position, transform.position + (transform.rotation * new Vector3(0, -f.z, 0)), Color.green);
            Debug.DrawLine(transform.position, transform.position + (transform.rotation * new Vector3(0, 0, f.x)), Color.blue);
        }

        if (showMoment) {
            var mo = aeroMoment * Plane.poundFootToNewtonMeter;

            float y = 1;

            if (inertiaTensor.y != 0) {
                y = inertiaTensor.y;
            }

            mo.y /= y;

            Debug.DrawLine(transform.position, transform.position + (transform.rotation * new Vector3(mo.y, 0, 0)), Color.red);
            Debug.DrawLine(transform.position, transform.position + (transform.rotation * new Vector3(0, -mo.z, 0)), Color.green);
            Debug.DrawLine(transform.position, transform.position + (transform.rotation * new Vector3(0, 0, mo.x)), Color.blue);
        }
    }

    void UpdateTrimmer() {
        if (!runTrimmer) return;

        float dt = trimmerSimTimeStep;
        float gravity = Vector3.Dot(Physics.gravity, Vector3.down) * Plane.metersToFeet;
        SimpleTrimmer.SimulatedState initialState = new SimpleTrimmer.SimulatedState {
            velocity = new Vector3(speed, 0, 0),
            alpha = 0,
        };

        SimpleTrimmer.SimulatedState state = simpleTrimmer.Trim(dt, trimmerSimTime, initialState, trimmerPitchRate * Mathf.Deg2Rad, gravity, trimmerPID, 0.35f);

        Quaternion rot = Quaternion.Euler(state.alpha, 0, 0);
        Vector3 dir = transform.rotation * rot * Vector3.forward;
        Debug.DrawRay(transform.position, dir * 10, Color.red);
    }
}
