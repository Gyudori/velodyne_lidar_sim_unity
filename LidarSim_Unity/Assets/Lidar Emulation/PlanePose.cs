using System.Collections;
using System.Collections.Generic;
using UnityEngine;

public class PlanePose : MonoBehaviour
{
    GameObject leftPlane;
    GameObject rightPlane;
    GameObject frontPlane;
    GameObject bottomPlane;
    // Start is called before the first frame update
    void Start()
    {
        // Get child gameobjects of planes
        leftPlane = transform.GetChild(0).gameObject;
        rightPlane = transform.GetChild(1).gameObject;
        frontPlane = transform.GetChild(2).gameObject;
        bottomPlane = transform.GetChild(3).gameObject;

        // Set position and rotation
        leftPlane.transform.position = new Vector3(-3.17264f, 0.652964f, 1.18361f);
        leftPlane.transform.rotation = new Quaternion(-0.034583f, 0f, -0.70652f, 0.70684f);

        rightPlane.transform.position = new Vector3(3.24706f, 0.559045f,    1.05111f);
        rightPlane.transform.rotation = new Quaternion(0.03487f, 0f, 0.70739f, 0.70596f);

        frontPlane.transform.position = new Vector3(0.188955f,    0.757378f,    5.3578f);
        frontPlane.transform.rotation = new Quaternion(-0.69497f, 0f, 0.037217f, 0.71807f);

        bottomPlane.transform.position = new Vector3(0.0191174f, - 0.436062f,   0.644547f);
        bottomPlane.transform.rotation = new Quaternion(0.013537f, 0f, -0.0022534f, 0.99991f);
    }

    // Update is called once per frame
    void Update()
    {
        
    }
    private void OnDestroy()
    {        
    }
}
