using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class Spawner : MonoBehaviour {
    [SerializeField]
    GameObject planePrefab;
    [SerializeField]
    PlayerController playerController;
    [SerializeField]
    Transform spawnGround;
    [SerializeField]
    Transform spawnAir;

    [SerializeField]
    bool spawnInAir;
    [SerializeField]
    float spawnInAirSpeed;

    GameObject activePlane;

    public bool SpawnInAir {
        get {
            return spawnInAir;
        }
        set {
            spawnInAir = value;
        }
    }

    void Start() {
        Spawn();
    }

    public void Spawn() {
        Transform spawnPoint = spawnGround;

        if (spawnInAir) {
            spawnPoint = spawnAir;
        }

        Spawn(spawnPoint);
    }

    void Spawn(Transform spawnPoint) {
        if (activePlane != null) {
            playerController.SetPlane(null);
            Destroy(activePlane);
        }

        activePlane = Instantiate(planePrefab);
        Transform t = activePlane.GetComponent<Transform>();
        t.SetPositionAndRotation(spawnPoint.position, spawnPoint.rotation);

        Plane plane = activePlane.GetComponent<Plane>();
        playerController.SetPlane(plane);
    }
}
