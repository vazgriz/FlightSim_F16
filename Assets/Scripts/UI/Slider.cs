using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Slider : MonoBehaviour {
    public enum SlideDirection {
        Right,
        Left,
        Up,
        Down
    }

    [SerializeField]
    SlideDirection slideDirection;
    [SerializeField]
    RectTransform slider;

    public void SetValue(float value) {
        if (slideDirection == SlideDirection.Right) {
            slider.anchorMin = new Vector2(value, 0);
            slider.anchorMax = new Vector2(value, 0);
        } else if (slideDirection == SlideDirection.Left) {
            slider.anchorMin = new Vector2(1 - value, 0);
            slider.anchorMax = new Vector2(1 - value, 0);
        } else if (slideDirection == SlideDirection.Up) {
            slider.anchorMin = new Vector2(0, value);
            slider.anchorMax = new Vector2(0, value);
        } else if (slideDirection == SlideDirection.Down) {
            slider.anchorMin = new Vector2(0, 1 - value);
            slider.anchorMax = new Vector2(0, 1 - value);
        }
    }
}
