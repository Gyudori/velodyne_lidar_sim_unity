using System.Collections;
using System.Collections.Generic;
using UnityEngine;
using System.IO;

namespace KETI
{
    public class ReadPose : MonoBehaviour
    {
        // TODO        
        // ListPose가 EndOfStream이면 게임을 destroy하도록 수정
        List<Vector3> lidarPos = new List<Vector3>();
        List<Quaternion> lidarQuat = new List<Quaternion>();

        int poseIndex = 0;
        public bool isPlay = false;
        double unityUTMOffsetX = 313592.8473d;
        double unityUTMOffsetY = 36d;
        double unityUTMOffsetZ = 4161038.8462d;        

        public void ReadCSVPose()
        {
            var reader = new StreamReader(File.OpenRead(@"./Assets/PosEul_500.csv"));
            
            while(!reader.EndOfStream)
            {
                var line = reader.ReadLine();
                var values = line.Split(',');

                double poseX = double.Parse(values[0]);
                double poseY = double.Parse(values[2]);
                double poseZ = double.Parse(values[1]);

                lidarPos.Add(new Vector3((float)(poseX - unityUTMOffsetX), (float)(poseY - unityUTMOffsetY), (float)(poseZ - unityUTMOffsetZ)));
                lidarQuat.Add(Quaternion.Euler(float.Parse(values[4]), float.Parse(values[5]), float.Parse(values[3]))); 
            }
        }


        // Start is called before the first frame update
        void Start()
        {
            ReadCSVPose();            
        }

        // Update is called once per frame
        void Update()
        {
            if (Input.GetKeyDown(KeyCode.Alpha1)) {
                isPlay = true;
            }

            if (isPlay) {
                if (poseIndex < lidarPos.Count)
                {
                    transform.position = lidarPos[poseIndex];
                    transform.rotation = lidarQuat[poseIndex];

                    poseIndex++;
                }
                else
                {
                    UnityEditor.EditorApplication.isPlaying = false;
                }
            }

        }
    }
}

