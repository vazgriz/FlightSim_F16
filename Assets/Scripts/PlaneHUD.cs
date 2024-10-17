using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem.XR;
using UnityEngine.UI;
using static UnityEngine.GraphicsBuffer;

public class PlaneHUD : MonoBehaviour {
    [SerializeField]
    float updateRate;
    [SerializeField]
    Color normalColor;
    [SerializeField]
    Color lockColor;
    [SerializeField]
    List<GameObject> helpDialogs;
    [SerializeField]
    Compass compass;
    [SerializeField]
    PitchLadder pitchLadder;
    [SerializeField]
    Slider throttleArrow;
    [SerializeField]
    Bar throttleBar;
    [SerializeField]
    Transform hudCenter;
    [SerializeField]
    Transform velocityMarker;
    [SerializeField]
    Text airspeed;
    [SerializeField]
    Text aoaIndicator;
    [SerializeField]
    Text aoaPredictedIndicator;
    [SerializeField]
    Text gforceIndicator;
    [SerializeField]
    Text gforcePredictedIndicator;
    [SerializeField]
    Text machIndicator;
    [SerializeField]
    Text altitude;
    [SerializeField]
    Bar healthBar;
    [SerializeField]
    Text healthText;
    [SerializeField]
    Transform targetBox;
    [SerializeField]
    Text targetName;
    [SerializeField]
    Text targetRange;
    [SerializeField]
    Transform missileLock;
    [SerializeField]
    Transform reticle;
    [SerializeField]
    RectTransform reticleLine;
    [SerializeField]
    RectTransform targetArrow;
    [SerializeField]
    RectTransform missileArrow;
    [SerializeField]
    float targetArrowThreshold;
    [SerializeField]
    float missileArrowThreshold;
    [SerializeField]
    float cannonRange;
    [SerializeField]
    float bulletSpeed;
    [SerializeField]
    GameObject aiMessage;
    [SerializeField]
    GameObject configMenu;
    [SerializeField]
    GameObject helpDialog;
    [SerializeField]
    Toggle enableEngine;
    [SerializeField]
    Toggle enableFCS;
    [SerializeField]
    Toggle rollControl;
    [SerializeField]
    Toggle pitchControl;
    [SerializeField]
    Toggle yawControl;
    [SerializeField]
    UnityEngine.UI.Slider centerOfGravitySlider;
    [SerializeField]
    Text centerOfGravityLabel;

    Plane plane;
    Transform planeTransform;
    new Camera camera;
    Transform cameraTransform;

    GameObject hudCenterGO;
    GameObject velocityMarkerGO;
    GameObject targetBoxGO;
    Image targetBoxImage;
    GameObject missileLockGO;
    Image missileLockImage;
    GameObject reticleGO;
    GameObject targetArrowGO;
    GameObject missileArrowGO;

    float lastUpdateTime;

    const float metersToKnots = 1.94384f;
    const float metersToFeet = 3.28084f;

    void Start() {
        hudCenterGO = hudCenter.gameObject;
        velocityMarkerGO = velocityMarker.gameObject;
        targetBoxGO = targetBox.gameObject;
        targetBoxImage = targetBox.GetComponent<Image>();
        missileLockGO = missileLock.gameObject;
        missileLockImage = missileLock.GetComponent<Image>();
        reticleGO = reticle.gameObject;
        targetArrowGO = targetArrow.gameObject;
        missileArrowGO = missileArrow.gameObject;
    }

    public void SetPlane(Plane plane) {
        this.plane = plane;

        if (plane == null) {
            planeTransform = null;
        } else {
            planeTransform = plane.GetComponent<Transform>();
        }

        if (compass != null) {
            compass.SetPlane(plane);
        }

        if (pitchLadder != null) {
            pitchLadder.SetPlane(plane);
        }

        ResetToggle(enableEngine);
        ResetToggle(enableFCS);
        ResetToggle(rollControl);
        ResetToggle(pitchControl);
        ResetToggle(yawControl);

        centerOfGravitySlider.value = plane.CenterOfGravityPosition * 100;
    }

    public void SetCamera(Camera camera) {
        this.camera = camera;

        if (camera == null) {
            cameraTransform = null;
        } else {
            cameraTransform = camera.GetComponent<Transform>();
        }

        if (compass != null) {
            compass.SetCamera(camera);
        }

        if (pitchLadder != null) {
            pitchLadder.SetCamera(camera);
        }
    }

    public void ToggleHelpDialogs() {
        foreach (var dialog in helpDialogs) {
            dialog.SetActive(!dialog.activeSelf);
        }
    }

    void ResetToggle(Toggle toggle) {
        if (toggle != null) {
            toggle.isOn = true;
        }
    }

    void UpdateVelocityMarker() {
        var velocity = planeTransform.forward;

        if (plane.LocalVelocity.sqrMagnitude > 1) {
            velocity = plane.Rigidbody.velocity;
        }

        var hudPos = TransformToHUDSpace(cameraTransform.position + velocity);

        if (hudPos.z > 0) {
            velocityMarkerGO.SetActive(true);
            velocityMarker.localPosition = new Vector3(hudPos.x, hudPos.y, 0);
        } else {
            velocityMarkerGO.SetActive(false);
        }
    }

    void UpdateAirspeed() {
        var speed = plane.LocalVelocity.z * metersToKnots;
        airspeed.text = string.Format("{0:0}", speed);
    }

    void UpdateAOA() {
        aoaIndicator.text = string.Format("{0:0.0} AOA", plane.AngleOfAttack * Mathf.Rad2Deg);
        aoaPredictedIndicator.text = string.Format("{0:0.0} AOA", plane.PredictedAngleOfAttack);
    }

    void UpdateGForce() {
        var gforce = plane.LocalGForce.y / 9.81f;
        gforceIndicator.text = string.Format("{0:0.0} G", gforce);
        var gforcePredicted = plane.PredictedLocalGForce.y / 9.81f;
        gforcePredictedIndicator.text = string.Format("{0:0.0} G", gforcePredicted);
    }

    void UpdateMach() {
        var mach = plane.Mach;
        machIndicator.text = string.Format("{0:0.0} M", mach);
    }

    void UpdateAltitude() {
        var altitude = plane.Rigidbody.position.y * metersToFeet;
        this.altitude.text = string.Format("{0:0}", altitude);
    }

    Vector3 TransformToHUDSpace(Vector3 worldSpace) {
        var screenSpace = camera.WorldToScreenPoint(worldSpace);
        return screenSpace - new Vector3(camera.pixelWidth / 2, camera.pixelHeight / 2);
    }

    void UpdateHUDCenter() {
        var rotation = cameraTransform.localEulerAngles;
        var hudPos = TransformToHUDSpace(cameraTransform.position + planeTransform.forward);

        if (hudPos.z > 0) {
            hudCenterGO.SetActive(true);
            hudCenter.localPosition = new Vector3(hudPos.x, hudPos.y, 0);
            hudCenter.localEulerAngles = new Vector3(0, 0, -rotation.z);
        } else {
            hudCenterGO.SetActive(false);
        }
    }

    void LateUpdate() {
        if (plane == null) return;
        if (camera == null) return;

        float power = Engine.InvertThrottleGear(plane.EnginePowerOutput);

        throttleArrow.SetValue(plane.Throttle);
        throttleBar.SetValue(power);

        if (!plane.Dead) {
            UpdateVelocityMarker();
            UpdateHUDCenter();
        } else {
            hudCenterGO.SetActive(false);
            velocityMarkerGO.SetActive(false);
        }

        UpdateAirspeed();
        UpdateAltitude();

        //update these elements at reduced rate to make reading them easier
        if (Time.time > lastUpdateTime + (1f / updateRate)) {
            UpdateAOA();
            UpdateGForce();
            UpdateMach();
            lastUpdateTime = Time.time;
        }
    }

    public void SetEnableEngine(bool value) {
        if (plane != null) {
            plane.EngineEnabled = value;
        }
    }

    public void SetEnableFCS(bool value) {
        if (plane != null) {
            plane.EnableFCS = value;
        }
    }

    public void SetEnableRollControl(bool value) {
        if (plane != null) {
            plane.EnableRollControl = value;
        }
    }

    public void SetEnablePitchControl(bool value) {
        if (plane != null) {
            plane.EnablePitchControl = value;
        }
    }

    public void SetEnableYawControl(bool value) {
        if (plane != null) {
            plane.EnableYawControl = value;
        }
    }

    public void SetCenterOfGravity(float value) {
        if (plane != null) {
            float effectiveValue = value / 100f;
            plane.CenterOfGravityPosition = effectiveValue;
            centerOfGravityLabel.text = string.Format("{0:.00}", effectiveValue);
        }
    }

    public void ToggleConfigMenu() {
        if (configMenu != null) {
            configMenu.SetActive(!configMenu.activeSelf);
        }
    }

    public void ToggleHelpMenu() {
        if (helpDialog != null) {
            helpDialog.SetActive(!helpDialog.activeSelf);
        }
    }
}
