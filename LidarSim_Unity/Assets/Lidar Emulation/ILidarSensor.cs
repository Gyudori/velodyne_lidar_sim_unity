using UnityEngine;

namespace KETI
{
    public interface ILidarSensor
    {
        /// <summary>
        /// Sensor 정보 세팅
        /// </summary>
        /// <param name="sensor_info">모든 Sensor의 공통 정보</param>
        /// <param name="lidar_config">Lidar Sensor 특정 정보</param>
        void Set_preference(SensorInfo sensor_info, LidarConfig lidar_config);

        /// <summary>
        /// 1Hz 기준으로 측정된 결과 값
        /// </summary>
        /// <returns></returns>
        LidarSensorData Get_sensor_data();

        /// <summary>
        /// Sensor 작동 제어
        /// </summary>
        /// <param name="on">on/off</param>
        void Operate(bool on);

        /// <summary>
        /// 가동 범위 시각화 제어(e.g. 가상환경에 Point Cloud를 겹쳐서 표시)
        /// </summary>
        /// <param name="on">on/off</param>
        void Visualize_operation(bool on);

        /// <summary>
        /// 측정 결과 시각화 제어
        /// </summary>
        /// <param name="on">on/off</param>
        /// <returns>결과물 확인용 RenderTexture 전달</returns>
        RenderTexture Visualize_point_cloud(bool on);
    }
}