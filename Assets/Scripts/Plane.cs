using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Plane : MonoBehaviour {
    public const float poundsForceToNewtons = 4.44822f;
    public const float metersToFeet = 3.28084f;
    public const float feetToMeters = 1.0f / metersToFeet;
    public const float poundFootToNewtonMeter = 1.35582f;
    public const float kilosToPounds = 2.20462f;
    public const float slugToKilo = 14.5939f;
    public const float footSquareToMeterSquare = 0.092903f;

    [SerializeField]
    float maxHealth;
    [SerializeField]
    float health;
    [SerializeField]
    float throttleSpeed;
    [SerializeField]
    Vector4 inertiaTensor;
    [SerializeField]
    float centerOfGravityPosition;

    [Header("Steering")]
    [SerializeField]
    bool enableFCS;
    [SerializeField]
    float inputLag;
    [SerializeField]
    float aileronRange;
    [SerializeField]
    float elevatorRange;
    [SerializeField]
    float rudderRange;
    [SerializeField]
    float aileronSpeed;
    [SerializeField]
    float elevatorSpeed;
    [SerializeField]
    float rudderSpeed;
    [SerializeField]
    Vector3 steeringSpeed;
    [SerializeField]
    AnimationCurve steeringSpeedCurve;
    [SerializeField]
    PIDController rollController;
    [SerializeField]
    PIDController pitchController;
    [SerializeField]
    PIDController yawController;
    [SerializeField]
    AnimationCurve rollPitchFactor;
    [SerializeField]
    AnimationCurve rollAOAFactor;
    [SerializeField]
    AnimationCurve rollPSchedule;
    [SerializeField]
    AnimationCurve rollDSchedule;

    [Header("Trimmer")]
    [SerializeField]
    float trimmerTimeStep;
    [SerializeField]
    float trimmerTime;
    [SerializeField]
    float predictedAoaLimitMax;
    [SerializeField]
    float predictedAoaLimitStrength;
    [SerializeField]
    float feedbackAoaLimitMax;
    [SerializeField]
    float feedbackAoaLimitStrength;
    [SerializeField]
    float stickPusherThreshold;
    [SerializeField]
    AnimationCurve stickPusherCurve;
    [SerializeField]
    float stickPusherMax;
    [SerializeField]
    float gLimit;
    [SerializeField]
    float gLimitPitch;
    [SerializeField]
    float gLimitStrength;

    [Header("Drag")]
    [SerializeField]
    AnimationCurve dragForward;
    [SerializeField]
    AnimationCurve dragBack;
    [SerializeField]
    AnimationCurve dragLeft;
    [SerializeField]
    AnimationCurve dragRight;
    [SerializeField]
    AnimationCurve dragTop;
    [SerializeField]
    AnimationCurve dragBottom;
    [SerializeField]
    Vector3 angularDrag;
    [SerializeField]
    float airbrakeDrag;

    [Header("Misc")]
    [SerializeField]
    List<Collider> landingGear;
    [SerializeField]
    PhysicMaterial landingGearBrakesMaterial;
    [SerializeField]
    List<GameObject> graphics;
    [SerializeField]
    GameObject damageEffect;
    [SerializeField]
    GameObject deathEffect;
    [SerializeField]
    float initialSpeed;

    new PlaneAnimation animation;

    float throttleInput;
    Vector3 controlInput;

    Vector3 lastVelocity;
    PhysicMaterial landingGearDefaultMaterial;
    Vector3 lastAngularAcceleration;

    AirData airData;

    AirDataComputer airDataComputer;
    Engine engine;
    Aerodynamics aerodynamics;
    SimpleTrimmer simpleTrimmer;
    Vector3 controlSurfaceTarget;

    public bool EnableRollControl { get; set; } = true;
    public bool EnablePitchControl { get; set; } = true;
    public bool EnableYawControl { get; set; } = true;

    public bool EnableFCS {
        get {
            return enableFCS;
        }
        set {
            enableFCS = value;
        }
    }

    public bool EngineEnabled {
        get {
            return engine.Enabled;
        }
        set {
            engine.Enabled = value;
        }
    }

    public float CenterOfGravityPosition {
        get {
            return centerOfGravityPosition;
        }
        set {
            centerOfGravityPosition = value;
        }
    }

    public ControlSurfaces ControlSurfaces { get; private set; }
    public ControlSurfaces ControlSurfacesNormalized { get; private set; }

    public float MaxHealth {
        get {
            return maxHealth;
        }
        set {
            maxHealth = Mathf.Max(0, value);
        }
    }

    public float Health {
        get {
            return health;
        }
        private set {
            health = Mathf.Clamp(value, 0, maxHealth);

            if (health <= MaxHealth * .5f && health > 0) {
                //damageEffect.SetActive(true);
            } else {
                //damageEffect.SetActive(false);
            }

            if (health == 0 && MaxHealth != 0 && !Dead) {
                Die();
            }
        }
    }

    public bool Dead { get; private set; }

    public Rigidbody Rigidbody { get; private set; }
    public float Throttle { get; private set; }
    public float EnginePowerCommand { get; private set; }
    public float EnginePowerOutput { get; private set; }
    public Vector3 EffectiveInput { get; private set; }
    public Vector3 Velocity { get; private set; }
    public Vector3 LocalVelocity { get; private set; }
    public Vector3 LocalGForce { get; private set; }
    public Vector3 PredictedLocalGForce { get; private set; }
    public Vector3 LocalAngularVelocity { get; private set; }
    public Vector3 RollPitchYaw { get; private set; }
    public float AngleOfAttack { get; private set; }
    public float PredictedAngleOfAttack { get; private set; }
    public float AngleOfAttackYaw { get; private set; }
    public bool AirbrakeDeployed { get; private set; }

    public float Mach {
        get {
            return airData.altitudeMach;
        }
    }

    public float AltitudeFeet { get; private set; }

    void Start() {
        animation = GetComponent<PlaneAnimation>();
        Rigidbody = GetComponent<Rigidbody>();

        if (landingGear.Count > 0) {
            landingGearDefaultMaterial = landingGear[0].sharedMaterial;
        }

        Rigidbody.velocity = Rigidbody.rotation * new Vector3(0, 0, initialSpeed);

        airDataComputer = new AirDataComputer();
        engine = new Engine();
        aerodynamics = new Aerodynamics();
        simpleTrimmer = new SimpleTrimmer(airDataComputer, aerodynamics, Rigidbody.mass * kilosToPounds / (-Physics.gravity.y * metersToFeet), inertiaTensor, elevatorRange, elevatorSpeed);

        EngineEnabled = true;

        // textbook moment of inertia is in slug-ft^2
        // need to convert to kg-m^2
        // 1 lb = 1 slug * 1 G
        Rigidbody.inertiaTensor = new Vector3(inertiaTensor.y, inertiaTensor.z, inertiaTensor.x) * poundFootToNewtonMeter;

        rollController.min = -aileronRange;
        rollController.max = aileronRange;
        pitchController.min = -elevatorRange;
        pitchController.max = elevatorRange;
        yawController.min = -rudderRange;
        yawController.max = rudderRange;
    }

    public void SetThrottleInput(float input) {
        if (Dead) return;
        throttleInput = input;
    }

    public void SetControlInput(Vector3 input) {
        if (Dead) return;
        controlInput = Vector3.ClampMagnitude(input, 1);
    }

    public void ApplyDamage(float damage) {
        Health -= damage;
    }

    void Die() {
        throttleInput = 0;
        Throttle = 0;
        Dead = true;

        //damageEffect.GetComponent<ParticleSystem>().Pause();
        deathEffect.SetActive(true);
    }

    void UpdateThrottle(float dt) {
        float target = 0;
        if (throttleInput > 0) target = 1;

        //throttle input is [-1, 1]
        //throttle is [0, 1]
        Throttle = Utilities.MoveTo(Throttle, target, throttleSpeed * Mathf.Abs(throttleInput), dt);

        AirbrakeDeployed = Throttle == 0 && throttleInput == -1;

        if (AirbrakeDeployed) {
            foreach (var lg in landingGear) {
                lg.sharedMaterial = landingGearBrakesMaterial;
            }
        } else {
            foreach (var lg in landingGear) {
                lg.sharedMaterial = landingGearDefaultMaterial;
            }
        }
    }

    void CalculateAngleOfAttack() {
        if (LocalVelocity.sqrMagnitude < 0.1f) {
            AngleOfAttack = 0;
            AngleOfAttackYaw = 0;
            return;
        }

        AngleOfAttack = Mathf.Atan2(-LocalVelocity.y, LocalVelocity.z);
        AngleOfAttackYaw = Mathf.Atan2(LocalVelocity.x, LocalVelocity.z);
    }

    void CalculateGForce(float dt) {
        var invRotation = Quaternion.Inverse(Rigidbody.rotation);
        var acceleration = (Velocity - lastVelocity) / dt;
        LocalGForce = invRotation * acceleration;
        lastVelocity = Velocity;
    }

    void UpdateAirData() {
        float speed = LocalVelocity.magnitude;  // m/s
        float speedFeet = speed * metersToFeet;
        AltitudeFeet = Rigidbody.position.y * metersToFeet;

        airData = airDataComputer.CalculateAirData(speedFeet, AltitudeFeet);
    }

    void CalculateState(float dt) {
        var invRotation = Quaternion.Inverse(Rigidbody.rotation);
        Velocity = Rigidbody.velocity;
        LocalVelocity = invRotation * Velocity;  //transform world velocity into local space
        LocalAngularVelocity = invRotation * Rigidbody.angularVelocity;  //transform into local space

        var euler = Rigidbody.rotation.eulerAngles;
        RollPitchYaw = new Vector3(
            Utilities.ConvertAngle360To180(euler.x),
            Utilities.ConvertAngle360To180(euler.y),
            Utilities.ConvertAngle360To180(euler.z)
        );

        CalculateAngleOfAttack();
        UpdateAirData();
    }

    void UpdateThrust(float dt) {
        engine.ThrottleCommand = Throttle;
        engine.Mach = Mach;
        engine.Altitude = AltitudeFeet;

        engine.Update(dt);
        EnginePowerCommand = engine.PowerCommand;
        EnginePowerOutput = engine.PowerOutput;

        Rigidbody.AddRelativeForce(new Vector3(0, 0, engine.Thrust * poundsForceToNewtons));
    }

    float ApplyLimiter(float value, float limit, float limitStrength) {
        if (limit <= 0) return 1;
        if (value < limit) return 1;

        float error = value - limit;
        error *= limitStrength;

        return limit / (limit + error);
    }

    float CalculateAOALimiter(float predictedAlpha) {
        float aoaPitchMult = 1.0f;

        aoaPitchMult *= ApplyLimiter(predictedAlpha, predictedAoaLimitMax, predictedAoaLimitStrength);

        float realAOA = AngleOfAttack * Mathf.Rad2Deg;
        aoaPitchMult *= ApplyLimiter(realAOA, feedbackAoaLimitMax, feedbackAoaLimitStrength);

        return aoaPitchMult;
    }

    float CalculateAOAPusher() {
        float bias = 0.0f;

        float aoa = AngleOfAttack * Mathf.Rad2Deg;

        if (aoa > stickPusherThreshold) {
            float error = aoa - stickPusherThreshold;
            bias = stickPusherCurve.Evaluate(error);
        }

        return Mathf.Min(stickPusherMax, bias);
    }

    float CalculateGLimiter(float predictedG, float gLimit) {
        float gForce = predictedG / 9.81f;
        float gPitchMult = ApplyLimiter(gForce, gLimit, gLimitStrength);

        return gPitchMult;
    }

    void UpdateControls(float dt) {
        float steeringSpeedFactor = steeringSpeedCurve.Evaluate(Mathf.Abs(LocalVelocity.z));
        Vector3 maxInput = new Vector3(-1, 0, 0);
        Vector3 maxAV = Vector3.Scale(maxInput, steeringSpeed * steeringSpeedFactor);

        // set control surface position directly from input
        Vector3 directTarget = Vector3.Scale(controlInput, new Vector3(-elevatorRange, -rudderRange, aileronRange));

        Vector3 av = LocalAngularVelocity * Mathf.Rad2Deg;

        rollController.P = rollPSchedule.Evaluate(Mathf.Max(0, LocalVelocity.z));
        rollController.D = rollDSchedule.Evaluate(Mathf.Max(0, LocalVelocity.z));

        float gravityFactor = Vector3.Dot(Physics.gravity, Rigidbody.rotation * Vector3.down);
        SimpleTrimmer.SimulatedState initialState = new SimpleTrimmer.SimulatedState {
            velocity = new Vector3(LocalVelocity.z, 0, -LocalVelocity.y) * metersToFeet,
            altitude = AltitudeFeet,
            alpha = AngleOfAttack * Mathf.Rad2Deg,
            pitchRate = LocalAngularVelocity.x
        };

        SimpleTrimmer.SimulatedState state = simpleTrimmer.Trim(
            trimmerTimeStep,
            trimmerTime,
            initialState,
            maxAV.x * Mathf.Deg2Rad,
            gravityFactor * metersToFeet,
            pitchController,
            centerOfGravityPosition
        );

        float predictedAlpha = state.maxAlpha;
        PredictedAngleOfAttack = Utilities.ConvertAngle360To180(predictedAlpha);

        float predictedG = -state.maxAccelerationZ * feetToMeters;
        PredictedLocalGForce = new Vector3(0, predictedG, 0);

        float aoaPitchMult = CalculateAOALimiter(predictedAlpha);
        float gLimit = gLimitPitch;
        if (controlInput.x > 0) {
            gLimit = this.gLimit;
        }
        float gPitchMult = CalculateGLimiter(predictedG, gLimit);

        float pitchMult = Mathf.Min(aoaPitchMult, gPitchMult);
        float rollMult = rollPitchFactor.Evaluate(Mathf.Abs(controlInput.x)) * rollAOAFactor.Evaluate(AngleOfAttack * Mathf.Rad2Deg);
        Vector3 stickPusher = new Vector3(CalculateAOAPusher(), 0, 0);

        Vector3 limitedInput = Vector3.Scale(controlInput, new Vector3(pitchMult, 1, rollMult)) + stickPusher;
        Vector3 targetAV = Vector3.Scale(limitedInput, steeringSpeed * steeringSpeedFactor);

        var accel = lastAngularAcceleration * Mathf.Rad2Deg * dt;

        Vector3 fcsTarget = new Vector3(
            -pitchController.Calculate(dt, av.x, accel.x, targetAV.x),
            -yawController.Calculate(dt, av.y, accel.y, targetAV.y),
            rollController.Calculate(dt, av.z, accel.z, targetAV.z)
        );

        controlSurfaceTarget = new Vector3(
            (EnableFCS && EnablePitchControl)  ? fcsTarget.x : directTarget.x,
            (EnableFCS && EnableYawControl) ? fcsTarget.y : directTarget.y,
            (EnableFCS && EnableRollControl)   ? fcsTarget.z : directTarget.z
        );

        var current = ControlSurfaces;

        ControlSurfaces = new ControlSurfaces(
            Utilities.MoveTo(current.elevator, controlSurfaceTarget.x, elevatorSpeed, dt, -elevatorRange, elevatorRange),
            Utilities.MoveTo(current.rudder,   controlSurfaceTarget.y, rudderSpeed,   dt, -rudderRange, rudderRange),
            Utilities.MoveTo(current.aileron,  controlSurfaceTarget.z, aileronSpeed,  dt, -aileronRange, aileronRange)
        );

        ControlSurfacesNormalized = new ControlSurfaces(
            ControlSurfaces.elevator / elevatorRange,
            ControlSurfaces.rudder   / rudderRange,
            ControlSurfaces.aileron  / aileronRange
        );
    }

    void UpdateAerodynamics(float alpha, float beta) {
        // angular velocity in radians/s
        var lav = new Vector3(
            LocalAngularVelocity.z,
            LocalAngularVelocity.x,
            LocalAngularVelocity.y
        );

        // AerodynamicState uses aerospace conventions
        // X = forward
        // Y = right
        // Z = down
        AerodynamicState currentState = new AerodynamicState {
            inertiaTensor = inertiaTensor,
            velocity = new Vector3(LocalVelocity.z, LocalVelocity.x, -LocalVelocity.y) * metersToFeet,
            angularVelocity = lav,
            airData = airData,
            alpha = alpha,
            beta = beta,
            xcg = centerOfGravityPosition,
            controlSurfaces = new ControlSurfaces(ControlSurfaces.elevator, ControlSurfaces.rudder, -ControlSurfaces.aileron)
        };

        var newState = aerodynamics.CalculateAerodynamics(currentState);
        var aeroForces = newState.force;
        var aeroAngularAcceleration = newState.angularAcceleration;

        // aeroForces in pounds
        var forces = new Vector3(aeroForces.y, -aeroForces.z, aeroForces.x) * poundsForceToNewtons;
        Rigidbody.AddRelativeForce(forces);

        // aeroAngularAcceleration changes angular velocity directly
        Vector3 avCorrection = new Vector3(aeroAngularAcceleration.y, aeroAngularAcceleration.z, aeroAngularAcceleration.x);
        Rigidbody.AddRelativeTorque(avCorrection, ForceMode.Acceleration);
        lastAngularAcceleration = avCorrection;
    }

    void FixedUpdate() {
        float dt = Time.fixedDeltaTime;

        //calculate at start, to capture any changes that happened externally
        CalculateState(dt);
        CalculateGForce(dt);

        //handle user input
        UpdateThrottle(dt);

        if (!Dead) {
            float alpha = AngleOfAttack * Mathf.Rad2Deg;
            float beta = AngleOfAttackYaw * Mathf.Rad2Deg;

            //apply updates
            UpdateThrust(dt);
            UpdateControls(dt);
            UpdateAerodynamics(alpha, beta);
        }

        //calculate again, so that other systems can read this plane's state
        CalculateState(dt);
    }

    void OnCollisionEnter(Collision collision) {
        for (int i = 0; i < collision.contactCount; i++) {
            var contact = collision.contacts[i];

            if (landingGear.Contains(contact.thisCollider)) {
                return;
            }

            Health = 0;

            Rigidbody.isKinematic = true;
            Rigidbody.position = contact.point;
            Rigidbody.rotation = Quaternion.Euler(0, Rigidbody.rotation.eulerAngles.y, 0);

            foreach (var go in graphics) {
                go.SetActive(false);
            }

            return;
        }
    }
}
