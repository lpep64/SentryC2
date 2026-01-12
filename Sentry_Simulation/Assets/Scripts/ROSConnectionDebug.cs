using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using System;

/// <summary>
/// Debug script to diagnose ROS TCP connection issues
/// Attach this to any GameObject in your scene to see detailed connection info
/// Auto-disables after max failed connection attempts
/// </summary>
public class ROSConnectionDebug : MonoBehaviour
{
    private ROSConnection ros;
    private float checkInterval = 2f;
    private float nextCheck = 0f;
    private int failedConnectionCount = 0;
    private int maxFailedAttempts = 4;
    private bool wasConnected = false;

    void Start()
    {
        Debug.Log("=== ROS CONNECTION DEBUG START ===");
        
        try
        {
            ros = ROSConnection.GetOrCreateInstance();
            
            if (ros == null)
            {
                Debug.LogError("❌ ROSConnection is NULL! This should never happen.");
                enabled = false;
                return;
            }

            Debug.Log("✓ ROSConnection instance obtained");
            LogConnectionDetails();
            
            // Force connect if not connected
            if (!ros.HasConnectionThread)
            {
                Debug.LogWarning("⚠️ No connection thread detected. Attempting manual connect...");
                ros.Connect();
            }
        }
        catch (Exception e)
        {
            Debug.LogError($"❌ Exception during ROS connection setup: {e.Message}");
            Debug.LogError($"Stack trace: {e.StackTrace}");
            enabled = false;
        }
        
        Debug.Log("=== ROS CONNECTION DEBUG END ===");
    }

    void Update()
    {
        if (Time.time >= nextCheck && ros != null)
        {
            nextCheck = Time.time + checkInterval;
            LogConnectionStatus();
        }
    }

    void LogConnectionDetails()
    {
        Debug.Log($"📍 ROS IP Address: {ros.RosIPAddress}");
        Debug.Log($"🔌 ROS Port: {ros.RosPort}");
        Debug.Log($"🚀 Connect on Start: {ros.ConnectOnStart}");
        Debug.Log($"🧵 Has Connection Thread: {ros.HasConnectionThread}");
        Debug.Log($"❗ Has Connection Error: {ros.HasConnectionError}");
        Debug.Log($"🌐 Protocol: ROS2 (assumed)");
    }

    void LogConnectionStatus()
    {
        if (ros == null)
        {
            enabled = false;
            return;
        }

        bool isConnected = ros.HasConnectionThread;
        string status = isConnected ? "✓ CONNECTED" : "✗ DISCONNECTED";
        string error = ros.HasConnectionError ? " (WITH ERRORS)" : "";
        
        // Track failed connections
        if (!isConnected)
        {
            failedConnectionCount++;
            Debug.LogWarning($"[{Time.time:F1}s] {status} - Failed attempt {failedConnectionCount}/{maxFailedAttempts}");
            
            if (failedConnectionCount >= maxFailedAttempts)
            {
                Debug.LogError($"❌ Max failed connection attempts ({maxFailedAttempts}) reached.");
                Debug.LogError("🛑 STOPPING DEBUG SCRIPT. Destroy this GameObject to stop connection errors.");
                this.enabled = false; // Stop Update() from running
                Destroy(this); // Remove the script component
                return;
            }
        }
        else
        {
            // Reset counter on successful connection
            if (!wasConnected)
            {
                Debug.Log($"[{Time.time:F1}s] {status} - Connection established!");
                failedConnectionCount = 0;
            }
            wasConnected = true;
            
            Debug.Log($"[{Time.time:F1}s] {status}{error}");
            
            if (ros.HasConnectionError)
            {
                Debug.LogWarning("Connection has errors. Check listener terminal for details.");
            }
        }
    }

    void OnDestroy()
    {
        Debug.Log("=== ROS CONNECTION DEBUG CLEANUP ===");
    }
}
