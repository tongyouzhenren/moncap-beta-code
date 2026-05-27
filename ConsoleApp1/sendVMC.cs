using System;
using System.Collections.Generic;
using System.Net.Sockets;
using System.Numerics;
using System.Runtime.InteropServices;
using System.Text;
using LucHeart.CoreOSC;
using System.Net;
using System.Net.Sockets;

namespace ConsoleApp1
{

    public class VmcSender
    {
        public static readonly string[] VmcBoneNames = new string[]
        {
            "Hips",               // IMU 0: 盆骨
            "Spine",              // IMU 1: 胸腔 (也可以改为 "Chest")
            "LeftUpperArm",       // IMU 2: 左大臂
            "RightUpperArm",      // IMU 3: 右大臂
            "LeftLowerArm",       // IMU 4: 左小臂
            "RightLowerArm",      // IMU 5: 右小臂
            "LeftHand",           // IMU 6: 左手掌
            "RightHand",          // IMU 7: 右手掌
            "LeftUpperLeg",       // IMU 8: 左大腿
            "RightUpperLeg",      // IMU 9: 右大腿
            "LeftLowerLeg",       // IMU 10: 左小腿
            "RightLowerLeg"       // IMU 11: 右小腿
        };

        private static UdpClient _udpClient;

        // 预先缓存 Apply 消息（每帧都要发，缓存起来节省性能）
        private static OscMessage _applyMessage = new OscMessage("/VMC/Ext/Blend/Apply");

        /// <summary>
        /// 1. 初始化 UDP 
        /// </summary>
        public static void InitializeUdp(string ip = "192.168.0.22", int port = 39539)
        {
            _udpClient = new UdpClient();
            _udpClient.Connect(ip, port);
        }

        /// <summary>
        /// 2. 发送单个面部表情数据 (Blendshape)
        /// </summary>
        public async Task SendBlendshapeAsync(string blendshapeName, float value)
        {
            var msg = new OscMessage("/VMC/Ext/Blend/Val", blendshapeName, value);
            byte[] bytes = msg.GetBytes();
            await _udpClient.SendAsync(bytes, bytes.Length);
        }

        /// <summary>
        /// 3. 发送骨骼数据 (如 Head 头部)
        /// </summary>
        public static async Task SendBonePosAsync(string boneName, float px, float py, float pz, float qx, float qy, float qz, float qw)
        {
            var msg = new OscMessage("/VMC/Ext/Bone/Pos", boneName, px, py, pz, qx, qy, qz, qw);
            byte[] bytes = msg.GetBytes();
            await _udpClient.SendAsync(bytes, bytes.Length);
        }

        /// <summary>
        /// 4. 发送 Apply 指令 (一帧的数据发送完毕后，必须调用此方法触发渲染)
        /// </summary>
        public static async Task SendApplyAsync()
        {
            byte[] bytes = _applyMessage.GetBytes();
            await _udpClient.SendAsync(bytes, bytes.Length);
        }

        /// <summary>
        /// 关闭连接
        /// </summary>
        public void Close()
        {
            _udpClient?.Close();
        }
    }

    internal class sendVMC
    {
        public static float[,] AccelBuffer = new float[12, 3];
        public static float[,] QuatBuffer = new float[12, 4];
        public static float[,] Pose = new float[12, 4];
        public static float[,] Tpose = new float[12, 4];
        static Quaternion[] FinalPose = new Quaternion[12];

        public class FilterLib
        {
            [DllImport("filter_lib.dll", CharSet = CharSet.Auto)]
            public static extern void test_func();

            [DllImport("filter_lib.dll", CharSet = CharSet.Auto)]
            public static extern void init_tpose(float[,] initPose);

            [DllImport("filter_lib.dll", CharSet = CharSet.Auto)]
            public static extern void updata_only_imu(float[,] ImuData, float[,] GetPose);
        }

        static int[] joint_parents = { -1, 0, 1, 1, 2, 3, 4, 5, 0, 0, 8, 9 };
        public static void RotationTrans(float[,] PoseData)
        {
            for (int i = 0; i < 12; i++)
            {
                // 假设算法输出 PoseData 顺序是 [W, X, Y, Z]
                float w = PoseData[i, 0];
                float x = PoseData[i, 1];
                float y = PoseData[i, 2];
                float z = PoseData[i, 3];

                // --- 坐标系转换 (Z-Up 右手系 -> Y-Up 左手系) ---
                // 映射规则：Unity.X = Sensor.X, Unity.Y = Sensor.Z, Unity.Z = Sensor.Y
                // 同时为了处理手性翻转，通常需要给 W 或某些轴取反
                Quaternion q = new Quaternion(x, z, y, -w);

                if (i == 0 || joint_parents[i] == -1)
                {
                    FinalPose[i] = q;
                }
                else
                {
                    int pIdx = joint_parents[i];
                    // 同样方式获取父节点
                    float pw = PoseData[pIdx, 0];
                    float px = PoseData[pIdx, 1];
                    float py = PoseData[pIdx, 2];
                    float pz = PoseData[pIdx, 3];
                    Quaternion parentQ = new Quaternion(px, pz, py, -pw);

                    // 计算局部旋转
                    FinalPose[i] = Quaternion.Inverse(parentQ) * q;
                }
            }
        }

        private static void GetData()
        {
            GetImuData._semaphore.Wait();

            GetImuData.Flag = false;

            Array.Copy(GetImuData.accel, AccelBuffer, AccelBuffer.Length);
            Array.Copy(GetImuData.quat, QuatBuffer, QuatBuffer.Length);

            GetImuData._semaphore.Release();
        }

        

        public static async Task Main()
        {
            _ = GetImuData.pipeStart();

            // 2. 设置等待时间（例如 5 秒）
            int waitSeconds = 5;
            Console.WriteLine($"\n=== 准备阶段 ===");
            Console.WriteLine($"请在 {waitSeconds} 秒内摆好 T-Pose 姿势并保持不动...");

            for (int i = waitSeconds; i > 0; i--)
            {
                Console.WriteLine($"倒计时: {i} ...");
                await Task.Delay(1000); // 异步等待 1 秒，不阻塞串口接收
            }

            for (int i = 0; i < 10; ++i)
            {
                GetData();
                for (int j = 0; j < 12; ++j)
                {
                    Tpose[j, 0] += QuatBuffer[j, 0];
                    Tpose[j, 1] += QuatBuffer[j, 1];
                    Tpose[j, 2] += QuatBuffer[j, 2];
                    Tpose[j, 3] += QuatBuffer[j, 3];
                }
            }

            for (int j = 0; j < 12; ++j)
            {
                Tpose[j, 0] /= 10;
                Tpose[j, 1] /= 10;
                Tpose[j, 2] /= 10;
                Tpose[j, 3] /= 10;
            }

            Console.WriteLine("Tpose Init Finish!");

            FilterLib.init_tpose(Tpose);

            Console.WriteLine("Tpose Init Complete!");

            VmcSender.InitializeUdp();

            Console.WriteLine("UDP Initialized!");

            while (true)
            {
                GetData();

                Console.WriteLine(AccelBuffer[1, 0]);
                
                FilterLib.updata_only_imu(QuatBuffer, Pose);
                
                RotationTrans(Pose);

                for (int i = 0; i < 12; i++)
                {
                    VmcSender.SendBonePosAsync(VmcSender.VmcBoneNames[i], 0, 0, 0, FinalPose[i].X, FinalPose[i].Y, FinalPose[i].Z, FinalPose[i].W).Wait();
                }

                VmcSender.SendApplyAsync().Wait();
            }
        }
    }
}
