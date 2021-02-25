using System;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace KETI
{
    public struct LidarSensorData
    {
        public SensorInfo Sensor_info;
        public LidarConfig Lidar_config;

        public IEnumerable<DetectionData> Detection_data;

        public LidarSensorData(SensorInfo sensor_info, LidarConfig lidar_config, IEnumerable<DetectionData> detection_data)
        {
            this.Sensor_info = sensor_info;
            this.Lidar_config = lidar_config;
            this.Detection_data = detection_data;
        }

        public override string ToString() => $"Id : {this.Sensor_info.Sensor_id}, Channel : {this.Lidar_config.Channels}, Size: {this.Detection_data.Count()}";
    }

    public struct DetectionData
    {
        public int Frame;
        public float Timestamp; 
        public Vector3 Position;
        public float Intensity;

        public DetectionData(int frame, float timestamp, Vector3 position, float intensity)
        {
            this.Frame = frame;
            this.Timestamp = timestamp;
            this.Position = position;
            this.Intensity = intensity;
        }
    }
}