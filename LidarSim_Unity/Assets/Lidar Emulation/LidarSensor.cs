//using KETI.SENSOR.DATA;
using UnityEngine;
using System.Collections.Generic;
using System;
using Unity.Jobs;
using Unity.Collections;
using Unity.Profiling;
using UnityEditor;
//using Unity.Mathematics;
using System.Security.Cryptography.X509Certificates;
using System.Text;
using System.IO;
using System.Diagnostics;

namespace KETI
{
    public class LidarSensor : MonoBehaviour, ILidarSensor
    {
        #region Set variable
        ProfilerMarker m_RaycastParallel = new ProfilerMarker("Lidar.UpdateParallel"); // ProfilerMarker : Performance marker used for profiling arbitrary code blocks.
        public Material m_PointCloudsMat;                                            // Point Clouds Material
        public GameObject lineDrawerPrefab;

        ////////////////////////////////////////////////////////////////////
        ///  센서 사양 변수
        ////////////////////////////////////////////////////////////////////

        // 입출력
        public SensorInfo m_SensorInfo;
        public LidarConfig m_LidarConfig;

        /////////////////////////////////////////////////////////////////////////
        /// Common 변수
        /////////////////////////////////////////////////////////////////////////

        public float m_RayRotation;               // 센서 회전을 위한 변수

        int frame = 1;                      
        float timeStamp = 0f;              
        int m_PointCount;                           // 포인트 갯수 (외부 사용 변수)

        public bool m_isRayCasting = false;          // RayCast 시작 flag
        public bool m_isSaveIntensity = false;      // 반사강도 저장 flag
        public bool m_isSaveDetectionData = false;  // 데이터 저장 flag
        public bool m_isDensePointCloud = false;    // 실측과 동일한 PPS로 빽빽한 PoC 생성 flag
        public bool m_isPreciseGeometricModel = false;
        public bool m_isPreciseRadiometricModel = false;

        bool m_isShowRayLine = true;                // 레이 화면에 그리기(Editor 전용)
        bool m_isSingleRayCast = false;             // 일반 Ray Cast 설정   

        public float fixedDeltatime = 0.02f;        // Lidar sensor 스캔 주기

        //공인평가용 변수
        public bool m_isSaveLog = true;
        List<int[]> logTable_100ms = new List<int[]>();
        string sensor_type;

        Stopwatch stw = new Stopwatch();

        ////////////////////////////////////////////////////////////////////
        ///  센서 동작 변수
        ////////////////////////////////////////////////////////////////////
        [Range(1, 2)]
        public int m_SensorID;                      // 센서 번호 1 - 16채널, 2 - 32채널
        public int m_RayChannelNum;                   // Y축 채널 갯수            
        public int m_PulseNumPerUpdate;                   // X축 채널 갯수
        public float m_VertStartAngle;             // Y축 채널 시작 각도            
        public float m_HorzStartAngle;             // X축 채널 시작 각도
        public float m_VertInterval;                // Y축 채널 레이 간격
        public float m_HorzInterval;                // X축 채널 레이 간격
        public float m_MaxDistance;                    //센서 측정 최대 거리
        public float m_HorzRotationLimit;           // 센서 회전 속도
        public float m_PointCloudSize;              // 포인트 클라우드 크기
        public float m_VertOffset;

        /////////////////////////////////////////////////////////////////////////
        /// Parallel 변수
        /////////////////////////////////////////////////////////////////////////

        NativeArray<RaycastCommand> m_RaycastCommand;      // 병렬 처리 명령어 저장 배열
        NativeArray<Vector2> m_RayAngles;           // 병렬 처리 명령어 저장 배열
        NativeArray<RaycastHit> m_Result;           // 병렬 처리 결과 저장 배열
        SaveRayDirection m_SaveRayDirection;                         // 병렬처리 Job Struct
        SaveDetectionData m_SaveDetectionData;                 // RayPoint 병렬 저장소

        JobHandle m_RayDirectionHandle;                      // 방향 및 RayCastCommand 설정하는 Job Handle
        JobHandle m_RayCastHandle;                  // 최종 Raycast 설정하는 JobHandle
        JobHandle m_SaveRayHandle;                  // Ray Point 저장하는 Jobhandle


        /////////////////////////////////////////////////////////////////////////
        /// PointClouds 변수
        /////////////////////////////////////////////////////////////////////////

        ComputeBuffer m_Buffer;                     // GPU data buffer, mostly for use with compute shaders.
        NativeArray<DetectionData> m_LidarDetectionData;            // 라이다 좌표계 기준 PoC : position = (distance, azimuth, verticalAngle)
        NativeArray<DetectionData> m_WorldDetectionData;            // 월드 좌표계 기준   PoC : position = (cartesian x, y, z)
        #endregion


        #region Variable
        // 기존 스크립트 변수

        /////////////////////////////////////////////////////////////////////////
        /// Interface 변수
        /////////////////////////////////////////////////////////////////////////      

        List<DetectionData> scanningData = new List<DetectionData>();
        List<DetectionData> latestFrame = new List<DetectionData>();
        List<DetectionData> outputFirstFrame = new List<DetectionData>();
        List<DetectionData> outputMappingResult = new List<DetectionData>();
        List<DetectionData> outputEveryFrame = new List<DetectionData>();
        LidarSensorData lidarSensorData;

        RenderTexture targetRenderTexture;

        string fileDir;
        List<string[]> dataTable;


        /////////////////////////////////////////////////////////////////////////
        /// Intensity 변수
        /////////////////////////////////////////////////////////////////////////

        double[,] intensityCoef16X6 = new double[16, 6] {
            {6.89E-05, 0.01358112, -0.458099931, 3.910060167, -10.7037611, 104.9941635 } ,
            {-0.007641683, 0.36550352, -6.362893105, 48.88152695, -165.1934509, 296.5238342 } ,
            { -0.003275766, 0.162957102, -2.904554129, 21.87664413, -68.80607605, 168.0875702-25} ,
            {0.001880599, -0.065678947, 0.892949879, -6.563269615, 20.48157501, 84.63182068-25} ,
            {-0.004353463, 0.190302685, -3.095044374, 23.03917694, -83.20815277, 200.1334839 } ,
            {-0.013622782, 0.554964781, -8.343720436, 57.24683762, -184.84552, 310.221283 } ,
            {-0.010462502, 0.505176783, -8.875789642, 68.9648056, -234.0849152, 373.6209106} ,
            {-0.001911886,  0.125920981, -2.761608124, 25.96287918, -110.0202866, 259.1989441} ,
            {-0.04811025, 1.833315134, -26.29503059, 175.3465271, -541.1307373, 704.6104736} ,
            {-0.022229461, 0.706064403, -8.679263115, 52.54673004, -166.1020508, 302.0780334} ,
            {-0.088931434, 2.879100084, -35.31969452, 202.9557953, -543.9476929, 638.0484619} ,
            {-0.015624537, 0.62781316, -9.340524673, 63.55469131, -202.8390656, 328.1264648} ,
            {0.040723115, -1.293815136, 15.55403137, -86.94377899, 215.1860046, -83.59525299 } ,
            {-0.42975384, 10.82821274, -103.8392487, 469.9481201, -1006.379211, 920.6851807} ,
            {-2.267603397, 45.43706894, -356.221405, 1364.966187, -2563.697266, 1988.038818 } ,
            {-11.75557613, 215.4252014, -1543.613037, 5397.80957, -9207.746094, 6233.609863}
        };

        #endregion

        #region Interface
        public void Set_preference(SensorInfo sensor_info, LidarConfig lidar_config)
        {
            //sensor setting
            m_SensorInfo = sensor_info;
            m_LidarConfig = lidar_config;

            //Lidar emulation variable setting
            m_RayChannelNum = lidar_config.Channels;
            m_PulseNumPerUpdate = (int)Math.Round(lidar_config.Firing_rate * fixedDeltatime);
            m_HorzStartAngle = 0f;
            m_VertInterval = lidar_config.Vertical_fov / (lidar_config.Channels - 1);
            m_VertStartAngle = -(lidar_config.Vertical_center_angle + (lidar_config.Vertical_fov / 2));
            m_HorzInterval = lidar_config.Horizontal_fov / (lidar_config.Firing_rate / lidar_config.Refresh_rate);
            m_MaxDistance = lidar_config.Range;
            m_HorzRotationLimit = lidar_config.Horizontal_fov * (lidar_config.Refresh_rate * fixedDeltatime);

            // Parallel Computing setting
            m_PointCount = m_PulseNumPerUpdate * m_RayChannelNum;

            m_Result = new NativeArray<RaycastHit>(m_PointCount, Allocator.Persistent);
            m_RaycastCommand = new NativeArray<RaycastCommand>(m_PointCount, Allocator.Persistent);
            m_RayAngles = new NativeArray<Vector2>(m_PointCount, Allocator.Persistent);

            m_Buffer = new ComputeBuffer(m_PointCount, 16);
            m_LidarDetectionData = new NativeArray<DetectionData>(m_PointCount, Allocator.Persistent);
            m_WorldDetectionData = new NativeArray<DetectionData>(m_PointCount, Allocator.Persistent);
        }

        public LidarSensorData Get_sensor_data()
        {
            lidarSensorData = new LidarSensorData(m_SensorInfo, m_LidarConfig, latestFrame);
            return lidarSensorData;
        }

        public void Operate(bool on)
        {
            m_isRayCasting = on;
        }

        public void Visualize_operation(bool on)
        {

        }

        public RenderTexture Visualize_point_cloud(bool on)
        {
            return targetRenderTexture;
        }
        #endregion

        #region Emulation
        void Start()
        {
            Time.fixedDeltaTime = fixedDeltatime;

            m_SensorInfo = new SensorInfo
            {
                Sensor_id = 1,
                Sensor_type = 1,
                Time_stamp = DateTime.Now,
                Mounting_position = transform.position,
                Mounting_rotation = transform.rotation
            };

            SelectLidarType();
            if(m_isPreciseGeometricModel)
                m_VertOffset = 0.000925f; // for precise geometric simulation
            stw.Start();
        }

        void SelectLidarType()
        {
            if (m_SensorID == 1) // VLP-16
            {
                if (m_isDensePointCloud)
                {
                    m_LidarConfig = new LidarConfig
                    {
                        Refresh_rate = 10,
                        Channels = 16,
                        Range = 100f,
                        Horizontal_fov = 360f,
                        Vertical_fov = 30f,
                        Vertical_center_angle = 0f,
                        Firing_rate = 18000
                    };
                }
                else
                {
                    m_LidarConfig = new LidarConfig
                    {
                        Refresh_rate = 10,
                        Channels = 16,
                        Range = 100f,
                        Horizontal_fov = 360f,
                        Vertical_fov = 30f,
                        Vertical_center_angle = 0f,
                        Firing_rate = 1000
                    };
                }
                sensor_type = "VLP-16";
                UnityEngine.Debug.Log("The selected lidar sensor is " + sensor_type);
            }
            else if (m_SensorID == 2) // HDL-32E
            {
                if (m_isDensePointCloud)
                {
                    m_LidarConfig = new LidarConfig
                    {
                        Refresh_rate = 10,
                        Channels = 32,
                        Range = 100f,
                        Horizontal_fov = 360f,
                        Vertical_fov = 41.33f,
                        Vertical_center_angle = 10f,
                        Firing_rate = 18000
                    };
                }
                else
                {
                    m_LidarConfig = new LidarConfig
                    {
                        Refresh_rate = 10,
                        Channels = 32,
                        Range = 100f,
                        Horizontal_fov = 360f,
                        Vertical_fov = 41.33f,
                        Vertical_center_angle = 10f,
                        Firing_rate = 1000
                    };
                }
                sensor_type = "HDL-32E";
                UnityEngine.Debug.Log("The selected lidar sensor is HDL-32E");
            }

            #region Logging part
            UnityEngine.Debug.Log("< Configuration > " + "\n"
                    + "Refresh_rate : " + m_LidarConfig.Refresh_rate + "\n"
                    + "Channel : " + m_LidarConfig.Channels + "\n"
                    + "Range : " + m_LidarConfig.Range + "\n"
                    + "Horizontal Field of View : " + m_LidarConfig.Horizontal_fov + "\n"
                    + "Vertical Field of View : " + m_LidarConfig.Vertical_fov + "\n"
                    + "Firing rate : " + m_LidarConfig.Firing_rate + "\n");
            Set_preference(m_SensorInfo, m_LidarConfig);
            #endregion
        }

        void Stop()
        {
            DisposeData();
        }

        void Update()
        {
            timeStamp += Time.fixedDeltaTime;

            StartRaycastWithPose();

            if (m_isRayCasting)
            {                
                m_RaycastParallel.Begin();
                RaycastParallel(timeStamp);
                //DrawPointClouds();
                m_RaycastParallel.End();

                if (m_isSaveIntensity)
                {
                    SaveIntensity();
                }
                
                scanningData.AddRange(m_LidarDetectionData); 
                outputEveryFrame.AddRange(m_LidarDetectionData);
                outputMappingResult.AddRange(m_WorldDetectionData);      
                
                m_RayRotation += m_HorzRotationLimit;
                
                if (m_RayRotation >= 360f)
                {
                    UpdateOneFrame();
                }
            }
        }

        private void StartRaycastWithPose()
        {
            ReadPose parent = GetComponentInParent<ReadPose>();

            if(parent != null)
            {
                if (parent.isPlay)
                {
                    if (!m_isRayCasting)
                        m_isRayCasting = true;
                }
            }
            
        }

        void UpdateOneFrame()
        {
            // Adding first frame data for test
            if (frame == 1)
            {
                outputFirstFrame.AddRange(outputMappingResult);
            }                

            m_RayRotation -= 360f;
            latestFrame = scanningData;

            scanningData = new List<DetectionData>();

            frame = frame + 1;

            #region Logging part
            UnityEngine.Debug.Log("Lidar sensor data generated per frame on " + stw.Elapsed + " : " + latestFrame.Count);
            if (m_isSaveLog)
            {
                int[] log_row_100ms = new int[2];
                log_row_100ms[0] = latestFrame.Count;
                log_row_100ms[1] = Convert.ToInt32(stw.ElapsedMilliseconds);

                logTable_100ms.Add(log_row_100ms);
            }
            #endregion
        }

        void SaveIntensity()
        {
            int i = 0;
            float reflectivity;
            float cosIncidence;
            float intensity;

            foreach (RaycastHit hit in m_Result)
            {
                if (hit.distance != 0)
                {
                    reflectivity = GetReflectivity(hit);
                    cosIncidence = m_WorldDetectionData[i].Intensity; // Calculate cosIncidence in jobsystem
                    if (m_isPreciseRadiometricModel)
                        intensity = reflectivity * Math.Abs(cosIncidence) * (float)ComputeIntensity(hit.distance, i);
                    else
                        intensity = reflectivity * Math.Abs(cosIncidence) / hit.distance / hit.distance;

                    if (intensity < 0 || intensity > 255)
                        intensity = 0;

                    // Save DetectionDate with calculated Intensity
                    m_LidarDetectionData[i] = new DetectionData(frame, m_LidarDetectionData[i].Timestamp, m_LidarDetectionData[i].Position, intensity);
                    m_WorldDetectionData[i] = new DetectionData(frame, m_WorldDetectionData[i].Timestamp, m_WorldDetectionData[i].Position, intensity);
                }
                else
                {
                    m_LidarDetectionData[i] = new DetectionData(frame, m_LidarDetectionData[i].Timestamp, m_LidarDetectionData[i].Position, 0);
                    m_WorldDetectionData[i] = new DetectionData(frame, m_WorldDetectionData[i].Timestamp, m_WorldDetectionData[i].Position, 0);
                }
                i++;
            }
            
        }

        void OnDestroy()
        {
            stw.Stop();
            if (m_isSaveDetectionData)
            { 
                ExportPointCloud(outputFirstFrame, m_RayChannelNum + "Channel_" + "_First_Frame.csv");
                ExportPointCloud(outputMappingResult, m_RayChannelNum + "Channel_Mapping_Result_" + frame.ToString() + "_Frames.csv");
                ExportPointCloud(outputEveryFrame, m_RayChannelNum + "Channel_LidarCS_" + frame.ToString() + "_Frames.csv");
            }

            if (m_isSaveLog)
            {
                ExportLog(logTable_100ms);
            }
            DisposeData();
        }

        

        void DisposeData()
        {
            if (m_Result.IsCreated)
            {
                m_Result.Dispose();
            }

            if (m_RaycastCommand.IsCreated)
            {
                m_RaycastCommand.Dispose();
            }

            m_Buffer?.Dispose();

            if (m_LidarDetectionData.IsCreated)
            {
                m_LidarDetectionData.Dispose();
            }

            if (m_WorldDetectionData.IsCreated)
            {
                m_WorldDetectionData.Dispose();
            }

            if (m_RayAngles.IsCreated)
            {
                m_RayAngles.Dispose();
            }
        }
        #endregion


        #region Parallel Logic

        void RaycastParallel(float ts)
        {
            // 1. 병렬처리 구조체 인스턴스화 및 데이터 채우기
            m_SaveRayDirection = new SaveRayDirection()
            {
                _ChannelNum = m_RayChannelNum,
                _HoriStartDeg = m_HorzStartAngle,
                _VertStartDeg = m_VertStartAngle,
                _HoriInterval = m_HorzInterval,
                _VertInterval = m_VertInterval,
                _BodyOrientation = transform.rotation,
                _Origin = transform.position,
                _MaxDistance = m_MaxDistance,
                _RayRotation = m_RayRotation,
                _VertOffset = m_VertOffset,
                _RaycastCommand = m_RaycastCommand,
                _RayAngles = m_RayAngles,
            };

            m_SaveDetectionData = new SaveDetectionData()
            {
                _RaycastHit = m_Result,
                _RaycastCommand = m_RaycastCommand,
                _RayAngles = m_RayAngles,
                _isSaveIntensity = m_isSaveIntensity,
                _LidarFrame = m_LidarDetectionData,
                _WorldFrame = m_WorldDetectionData,
                _time = ts,
                _frame = frame,
            };

            // 2. 방향각(Direction) 계산을 위한 병렬처리 스케쥴 등록
            //      mDIrP 내부 변수 __RayCast에 RaycastCommand의 원점, 방향 등 저장
            m_RayDirectionHandle = m_SaveRayDirection.Schedule(m_PointCount, 256);

            // 3. RaycastCommand 병렬처리 스케쥴 등록 
            //      발사할 방향이 정의된 m_Command(Native array of RaycastCommands)는 m_DirHandle에 의존함
            m_RayCastHandle = RaycastCommand.ScheduleBatch(m_RaycastCommand, m_Result, 256, m_RayDirectionHandle);

            // 4. Point 저장 스케쥴 등록
            //      m_SaveRayPoint에서 저장에 사용할 입력값 _Input, _Input2는 m_RayCastHandle에 의존함
            //      즉, Execute 될 때 m_RayCastHandle에 저장된 m_Result, m_Command가 _Input으로 입력됨
            //      최종적으로 m_PointData에 결과 저장
            m_SaveRayHandle = m_SaveDetectionData.Schedule(m_PointCount, 256, m_RayCastHandle);

            // 모든 병렬처리 완료 설정
            // 완료 설정부분이 너무 빠르면 모든 정보가 전부 처리되지 않는다.
            // LateUpdate혹은 다음 프레임에 처리하는 방법을 염두에 두어야 한다.
            m_SaveRayHandle.Complete();
        }

        struct SaveRayDirection : IJobParallelFor
        {          
            // input
            public int _ChannelNum;
            public float _HoriStartDeg;
            public float _VertStartDeg;
            public float _HoriInterval;
            public float _VertInterval;
            public Quaternion _BodyOrientation;
            public Vector3 _Origin;
            public float _MaxDistance;
            public float _RayRotation;
            public float _VertOffset;

            public int _HoriIndex;
            public int _VertIndex;

            // output
            public NativeArray<RaycastCommand> _RaycastCommand;
            public NativeArray<Vector2> _RayAngles;

            public void Execute(int i)
            {
                _HoriIndex = i / _ChannelNum;
                _VertIndex = i % _ChannelNum;

                float HoriAngle = _HoriStartDeg + _HoriIndex * _HoriInterval + _RayRotation;
                float VertAngle = -_VertStartDeg - _VertIndex * _VertInterval;                

                Vector3 direction = (_BodyOrientation * Quaternion.Euler(VertAngle, HoriAngle, 0f) * new Vector3(0, 0f, 1f)).normalized;
                if(_VertOffset != 0)
                    OffsetOrigin(_VertIndex);                 
                _RaycastCommand[i] = new RaycastCommand(_Origin, direction, _MaxDistance);
                _RayAngles[i] = new Vector2(HoriAngle, VertAngle);
            }

            private void OffsetOrigin(int verticalIndex)
            {
                float vertCoord;
                if(verticalIndex >= _ChannelNum/2)
                {
                    verticalIndex = verticalIndex - _ChannelNum/2 + 1;
                    vertCoord = verticalIndex * _VertOffset;
                }
                else
                {
                    verticalIndex = _ChannelNum/2 - verticalIndex;
                    vertCoord = verticalIndex * _VertOffset;
                }
                _Origin = _BodyOrientation * new Vector3(0, vertCoord, 0);                
            }
        }

        struct SaveDetectionData : IJobParallelFor
        {
            // input
            public NativeArray<RaycastHit> _RaycastHit;
            public NativeArray<RaycastCommand> _RaycastCommand;
            public NativeArray<Vector2> _RayAngles;
            public bool _isSaveIntensity;

            public float _time;
            public float _cosineIncidence;
            public int _frame;

            //output
            public NativeArray<DetectionData> _LidarFrame; 
            public NativeArray<DetectionData> _WorldFrame; // Detection data를 저장하기 위한 변수
            

            public void Execute(int i)
            {
                if (_isSaveIntensity)
                {
                    if (_RaycastHit[i].distance != 0)
                    {
                        _cosineIncidence = _RaycastHit[i].normal.x * _RaycastCommand[i].direction.x + _RaycastHit[i].normal.y * _RaycastCommand[i].direction.y + _RaycastHit[i].normal.z * _RaycastCommand[i].direction.z;
                    }
                }
                else
                    _cosineIncidence = 0f;
                
                _LidarFrame[i] = new DetectionData(_frame, _time, new Vector3(_RaycastHit[i].distance, _RayAngles[i].x, _RayAngles[i].y), _cosineIncidence); 
                _WorldFrame[i] = new DetectionData(_frame, _time, _RaycastHit[i].point, _cosineIncidence);
            }
        }

        #endregion

        #region Drawing
        //void DrawPointClouds()
        //{
        //    // Set the buffer with values from an array.
        //    m_Buffer.SetData(m_PointData);

        //    m_PointCloudsMat.SetBuffer("_PointCloud", m_Buffer);
        //    m_PointCloudsMat.SetMatrix("_LocalToWorld", Matrix4x4.identity);
        //    m_PointCloudsMat.SetFloat("_Size", m_PointCloudSize);
        //    m_PointCloudsMat.SetColor("_Color", Color.red);
        //    Graphics.DrawProcedural(m_PointCloudsMat, new Bounds(transform.position, m_Distance * Vector3.one), MeshTopology.Points, m_Buffer.count);
        //}

        private void OnDrawGizmos()
        {
            if (m_isShowRayLine)
            {
                Gizmos.color = Color.green;

                if (!m_isSingleRayCast)
                {
                    for (int i = 0; i < m_PointCount; i++)
                    {
                        if (m_Result.Length != 0 && m_Result[i].point != Vector3.zero)
                            Gizmos.DrawLine(this.transform.position, m_Result[i].point);
                    }
                }
                //else
                //{
                //   for (int i = 0; i < m_PointCount; i++)
                //   {
                //      if (m_ResultHit.Length != 0 && m_ResultHit[i].point != Vector3.zero)
                //         Gizmos.DrawLine(this.transform.position, m_ResultHit[i].point);
                //   }
                //}
            }
        }
        #endregion



        #region Output

        // 아래와 같이 m_Result[index]로 접근 가능합니다.
        // 복사시에 프레임 드랍이 있을수 있습니다.
        public Vector3 GetPointData(int index)
        {
            return m_Result[index].point;
        }

        public float GetDistance(int index)
        {
            return m_Result[index].distance;
        }
        #endregion

        #region Exporting Functions        

        private void ExportPointCloud(List<DetectionData> pointList, string fileName)
        {
            dataTable = new List<string[]>();

            //int time, Vector3 coordinate, int laserId, float azimuth
            foreach (DetectionData point in pointList)
            {
                string[] rows = new string[6];
                rows[0] = point.Frame.ToString();
                rows[1] = point.Timestamp.ToString();
                rows[2] = point.Position.x.ToString("0.000000");
                rows[3] = point.Position.y.ToString("0.000000");
                rows[4] = point.Position.z.ToString("0.000000");
                rows[5] = point.Intensity.ToString("0.000000");
                dataTable.Add(rows);
            }

            StringBuilder sb = new StringBuilder();
            int length = dataTable.Count;
            string delimiter = " , ";

            sb.AppendLine("Frame, TimeStamp, X, Y, Z, intensity");
            for (int r = 0; r < length; r++)
            {
                sb.AppendLine(string.Join(delimiter, dataTable[r]));
            }
            fileDir = "../output_sample/";
            StreamWriter outputstream = System.IO.File.CreateText(fileDir + fileName);
            outputstream.WriteLine(sb);
            outputstream.Close();
        }        

        public void ExportLog(List<int[]> Log_100ms)
        {
            string fullPath = "Assets/";
            string filename = "log";
            if (false == File.Exists(fullPath))
            {
                var file = File.CreateText(fullPath + m_RayChannelNum + "Channel_" + filename + ".txt");
                file.Close();
            }

            StreamWriter sw = new StreamWriter(fullPath + m_RayChannelNum + "Channel_" + filename + ".txt");
            //string fullPath = "Assets/";
            //string filename = DateTime.Now.ToString("yyyy/MM/dd-hh-mm-ss");
            //string foldernamewithdatetime = DateTime.Now.ToString("yyyy/MM/dd");
            //Directory.CreateDirectory(fullPath + foldernamewithdatetime + "/");
            //if (false == File.Exists(fullPath + foldernamewithdatetime + "/"))
            //{
            //    var file = File.CreateText(fullPath + foldernamewithdatetime + "/" + filename + ".txt");
            //    file.Close();
            //}

            //StreamWriter sw = new StreamWriter(fullPath + foldernamewithdatetime + "/" + filename + ".txt");

            sw.WriteLine("Simulator Start on " + m_SensorInfo.Time_stamp.ToString("yyyy/MM/dd hh:mm:ss"));
            sw.WriteLine("The selected lidar sensor is " + sensor_type);
            sw.WriteLine("<Configuration>");
            sw.WriteLine("Refresh_rate : " + m_LidarConfig.Refresh_rate);
            sw.WriteLine("Channel : " + m_LidarConfig.Channels);
            sw.WriteLine("Range : " + m_LidarConfig.Range);
            sw.WriteLine("Horizontal Field of View : " + m_LidarConfig.Horizontal_fov);
            sw.WriteLine("Vertical Field of View : " + m_LidarConfig.Vertical_fov);
            sw.WriteLine("Firing rate : " + m_LidarConfig.Firing_rate);
            sw.WriteLine();

            sw.WriteLine("Lidar sensor data generated per frame for " + Log_100ms[0][1] + "ms : " + Log_100ms[0][0] + " points");

            int sum_point = Log_100ms[0][0];
            for (int i = 1; i < Log_100ms.Count; i++)
            {
                int interval = Log_100ms[i][1] - Log_100ms[i - 1][1];
                sum_point += Log_100ms[i][0];

                sw.WriteLine("Lidar sensor data generated per frame for " + interval + "ms : " + Log_100ms[0][0] + " points");
                if (i == (Log_100ms.Count - 1))
                {
                    sw.WriteLine("Total generated points : " + sum_point + " points");
                    sw.WriteLine("Processing Time : " + Log_100ms[i][1] + "ms");
                    int emulation_speed = sum_point / (Log_100ms[i][1] / 1000);
                    sw.WriteLine("모사속도(point/s) : " + emulation_speed);
                }
            }


            sw.Flush();
            sw.Close();
        }
        #endregion

        #region Intensity Functions

        double ComputeIntensity(double distance, int pointIndex)
        {
            int channel;
            double intensity;
            channel = 15 - pointIndex % 16; 
            //matlab에서 채널이 위에서부터 1번 unity 아래서부터 1번            

            distance = SetDistanceFormulaRange(channel, distance);            

            intensity = (intensityCoef16X6[channel, 0] * Math.Pow(distance, 5d)) + 
                (intensityCoef16X6[channel, 1] * Math.Pow(distance, 4d)) + 
                (intensityCoef16X6[channel, 2] * Math.Pow(distance, 3d)) + 
                (intensityCoef16X6[channel, 3] * Math.Pow(distance, 2d)) + 
                (intensityCoef16X6[channel, 4] * distance) + 
                intensityCoef16X6[channel, 5];


            return intensity;
        }

        private double SetDistanceFormulaRange(int channel, double distance)
        {
            int narrowRangeChannelStart = 14;
            double narrowRangeMax = 5d;
            double rangeMax = 8d;
            double rangeMin = 2.5d;

            if(channel >= narrowRangeChannelStart)
            {
                if (distance > narrowRangeMax)
                    distance = narrowRangeMax;
            }
            else
            {
                if (distance > rangeMax)
                    distance = rangeMax;
            }

            if (distance < rangeMin)
                distance = rangeMin;

            return distance;
        }

        public float GetReflectivity(RaycastHit hit)
        {
            float reflectivity;
            Color color;

            if (hit.transform.GetComponent<Renderer>() != null)
            {
                Renderer rend = hit.transform.GetComponent<Renderer>();
                MeshCollider meshCollider = hit.collider as MeshCollider;

                if (rend.material.mainTexture == null)
                {
                    color = rend.material.color;
                }
                else
                {
                    Texture2D tex = rend.material.mainTexture as Texture2D;

                    // Make texture readable(process only once for a texture)
                    if (!rend.material.mainTexture.isReadable)
                    {
                        SetTextureImporterFormat(tex, true);
                    }

                    Vector2 pixelUV = hit.textureCoord;
                    pixelUV.x *= tex.width;
                    pixelUV.y *= tex.height;

                    Vector2 tiling = rend.material.mainTextureScale;
                    color = tex.GetPixel(Mathf.FloorToInt(pixelUV.x * tiling.x), Mathf.FloorToInt(pixelUV.y * tiling.y));
                }
                reflectivity = color.grayscale;

                return reflectivity;
            }
            else
                return 0;
        }

        public static void SetTextureImporterFormat(Texture2D texture, bool isReadable)
        {
            if (null == texture) return;

            string assetPath = AssetDatabase.GetAssetPath(texture);
            var tImporter = AssetImporter.GetAtPath(assetPath) as TextureImporter;
            if (tImporter != null)
            {
                tImporter.textureType = TextureImporterType.Default;

                tImporter.isReadable = isReadable;

                AssetDatabase.ImportAsset(assetPath);
                AssetDatabase.Refresh();
            }
        }

        
        # endregion

    }
}