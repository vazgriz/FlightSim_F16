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

    Vector3 rollPitchYaw;
    float alpha;

    new Transform transform;
    AirDataComputer airDataComputer;
    Aerodynamics aerodynamics;
    Engine engine;

    void Start() {
        transform = GetComponent<Transform>();

        airDataComputer = new AirDataComputer();
        aerodynamics = new Aerodynamics();
        engine = new Engine();
    }

    void FixedUpdate() {
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
            rotation = new Vector3(rollPitchYaw.z, rollPitchYaw.x, rollPitchYaw.y),
            angularVelocity = new Vector3(0, 0, 0),
            airData = airData,
            altitude = altitude,
            alpha = alpha,
            beta = rollPitchYaw.y,
            controlSurfaces = new Vector3(0, 0, 0)
        };

        var forces = aerodynamics.CalculateAerodynamics(state);
        var aeroForces = forces.force;
        var aeroMoment = forces.moment;

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
}
