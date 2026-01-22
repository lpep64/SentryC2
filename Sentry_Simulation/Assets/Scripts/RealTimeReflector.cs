using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using SensorJointState = RosMessageTypes.Sensor.JointStateMsg;
using System.Collections.Generic;

public class RealTimeReflector : MonoBehaviour
{
    // CONFIGURATION
    public string rosTopic = "/joint_states";
    public List<ArticulationBody> unityJoints; // Drag your robot joints here in Inspector

    private ROSConnection ros;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.Subscribe<SensorJointState>(rosTopic, UpdateRobotPose);
        Debug.Log("SentryC2: Listening for Physical Robot...");
    }

    void UpdateRobotPose(SensorJointState msg)
    {
        // ROS uses Radians. Unity ArticulationBody uses Degrees.
        // We must map the incoming array order to our Unity joint list.
        
        // HARDCODED MAPPING for Niryo Ned 2 (Standard Order)
        // Joint 1 -> Index 0
        // Joint 2 -> Index 1
        // ...
        
        if (unityJoints.Count != msg.position.Length) {
            Debug.LogWarning($"Joint Count Mismatch! Unity: {unityJoints.Count}, ROS: {msg.position.Length}");
            return;
        }

        for (int i = 0; i < unityJoints.Count; i++)
        {
            // Convert Radians to Degrees
            float degrees = (float)msg.position[i] * Mathf.Rad2Deg;
            
            // Apply to Articulation Body
            ArticulationBody joint = unityJoints[i];
            var drive = joint.xDrive;
            drive.target = degrees;
            joint.xDrive = drive;
        }
    }
}
