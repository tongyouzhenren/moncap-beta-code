#ifndef FILTER_H
#define FILTER_H

// 1. 使用 constexpr 代替 #define (C++17 标准)
constexpr int BUFFER_SIZE = 2;
constexpr int NODE_NUM = 12;
constexpr int VIDEO_FRAME_SPEED = 60;
constexpr int DT2 = VIDEO_FRAME_SPEED * VIDEO_FRAME_SPEED;
constexpr float SCORE_NODE = 0.5f;

// 2. 函数声明
// 注意：C++ 中传递多维数组建议写清楚维度，或者使用指针
extern "C" __declspec(dllexport) void init_tpose(float quat[NODE_NUM][4]);

extern "C" __declspec(dllexport) void updata_only_imu(const float imu_data[NODE_NUM][4], float get_pose[NODE_NUM][4]);

// 修正了拼写错误和缺少的分号
extern "C" __declspec(dllexport) void updata_with_vedio(float imu_data[NODE_NUM][4], float accel[NODE_NUM][3], float position_2D[NODE_NUM][2], float score[NODE_NUM], float get_pose[NODE_NUM][4]);


extern "C" __declspec(dllexport) void test_func(void);

#endif