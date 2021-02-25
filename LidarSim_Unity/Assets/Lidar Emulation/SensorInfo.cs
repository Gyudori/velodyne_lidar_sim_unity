using System;
using UnityEngine;

namespace KETI
{
    public struct SensorInfo
    {
        public ulong Sensor_id;

        public ulong Sensor_type; //Ãß°¡

        public DateTime Time_stamp;

        public Vector3 Mounting_position;
        public Quaternion Mounting_rotation;
    }
}