using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using UnityEngine.InputSystem;

public class PlayerController : MonoBehaviour {
    [SerializeField]
    new Camera camera;
    [SerializeField]
    Plane plane;
    [SerializeField]
    PlaneHUD planeHUD;

    Vector3 controlInput;
    PlaneCamera planeCamera;

    void Awake() {
        planeCamera = GetComponent<PlaneCamera>();
    }

    void Start() {
        if (plane != null) {
            SetPlane(plane);
        }
    }

    public void SetPlane(Plane plane) {
        this.plane = plane;

        if (plane != null && planeHUD != null) {
            planeHUD.SetPlane(plane);
            planeHUD.SetCamera(camera);
        }

        planeCamera.SetPlane(plane);
    }
    public void OnToggleHelp(InputAction.CallbackContext context) {
        if (plane == null) return;

        if (context.phase == InputActionPhase.Performed) {
            planeHUD.ToggleHelpDialogs();
        }
    }

    public void SetThrottleInput(InputAction.CallbackContext context) {
        if (plane == null) return;
        //if (aiController.enabled) return;

        plane.SetThrottleInput(context.ReadValue<float>());
    }

    public void OnRollPitchInput(InputAction.CallbackContext context) {
        if (plane == null) return;

        var input = context.ReadValue<Vector2>();
        controlInput = new Vector3(input.y, controlInput.y, -input.x);
    }

    public void OnYawInput(InputAction.CallbackContext context) {
        if (plane == null) return;

        var input = context.ReadValue<float>();
        controlInput = new Vector3(controlInput.x, input, controlInput.z);
    }

    public void OnCameraInput(InputAction.CallbackContext context) {
        if (plane == null) return;

        var input = context.ReadValue<Vector2>();
        planeCamera.SetInput(input);
    }

    public void OnFlapsInput(InputAction.CallbackContext context) {
        if (plane == null) return;

        if (context.phase == InputActionPhase.Performed) {
            plane.ToggleFlaps();
        }
    }

    void Update() {
        if (plane == null) return;

        plane.SetControlInput(controlInput);
    }
}
