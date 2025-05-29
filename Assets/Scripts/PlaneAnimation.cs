using System;
using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlaneAnimation : MonoBehaviour {
    [Serializable]
    public class ControlSurface {
        public string name;
        public float min;
        public float max;
        public Vector3 influence;

        int animationHash;

        public void Init() {
            animationHash = Animator.StringToHash(name);
        }

        public void Update(Animator animator, ControlSurfaces surfaces) {
            Vector3 s = new Vector3(surfaces.elevator, surfaces.rudder, surfaces.aileron);
            Vector3 mix = Vector3.Scale(influence, s);
            float value = mix.x + mix.y + mix.z;
            float t = Mathf.InverseLerp(min, max, value);
            animator.SetFloat(animationHash, t);
        }
    }

    [SerializeField]
    List<GameObject> afterburnerGraphics;
    [SerializeField]
    float afterburnerThreshold;
    [SerializeField]
    float afterburnerMinSize;
    [SerializeField]
    float afterburnerMaxSize;
    [SerializeField]
    float deflectionSpeed;
    [SerializeField]
    List<ControlSurface> controlSurfaces;
    [SerializeField]
    List<GameObject> missileGraphics;

    Plane plane;
    Animator animator;
    List<Transform> afterburnersTransforms;
    float airbrakePosition;
    float flapsPosition;

    void Start() {
        plane = GetComponent<Plane>();
        animator = GetComponent<Animator>();

        afterburnersTransforms = new List<Transform>();

        foreach (var go in afterburnerGraphics) {
            afterburnersTransforms.Add(go.GetComponent<Transform>());
        }

        foreach (var controlSurface in controlSurfaces) {
            controlSurface.Init();
        }
    }

    public void ShowMissileGraphic(int index, bool visible) {
        missileGraphics[index].SetActive(visible);
    }

    void UpdateAfterburners() {
        float throttle = plane.EnginePowerOutput;
        float afterburnerT = Mathf.Clamp01(Mathf.InverseLerp(afterburnerThreshold, 100, throttle));
        float size = Mathf.Lerp(afterburnerMinSize, afterburnerMaxSize, afterburnerT);

        if (throttle >= afterburnerThreshold) {
            for (int i = 0; i < afterburnerGraphics.Count; i++) {
                afterburnerGraphics[i].SetActive(true);
                afterburnersTransforms[i].localScale = new Vector3(size, size, size);
            }
        } else {
            for (int i = 0; i < afterburnerGraphics.Count; i++) {
                afterburnerGraphics[i].SetActive(false);
            }
        }
    }

    void UpdateControlSurfaces(float dt) {
        var surfaces = plane.ControlSurfacesNormalized;

        foreach (var controlSurface in controlSurfaces) {
            controlSurface.Update(animator, surfaces);
        }
    }

    void UpdateAirbrakes(float dt) {
        var target = plane.AirbrakeDeployed ? 1 : 0;

        airbrakePosition = Utilities.MoveTo(airbrakePosition, target, deflectionSpeed, dt);
    }

    void Update() {
        float dt = Time.deltaTime;

        if (plane.Dead) return;

        UpdateAfterburners();
        UpdateControlSurfaces(dt);
        UpdateAirbrakes(dt);
    }
}
