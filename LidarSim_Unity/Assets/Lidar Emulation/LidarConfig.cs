namespace KETI
{
    public struct LidarConfig
    {
        public int Refresh_rate; // Hz

        public int Channels;
        public float Range; // m

        public float Horizontal_fov;
        public float Vertical_fov;

        // 추가
        public float Vertical_center_angle; // vertical center angle of laser bundle
        public int Firing_rate; // laser pulses firing per second  

        // 제거 가능
        //public float Horizontal_resolution; // Horizontal_resolution = Refresh_rate*360/Firing_rate (rotating lidar)
        //public float Vertical_resolution; // Vertical_resolution = Vertical_fov/Channels
    }
}