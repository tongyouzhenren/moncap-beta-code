#include <boost/program_options.hpp>

// GTSAM related includes.
#include <gtsam/inference/Symbol.h>
#include <gtsam/navigation/CombinedImuFactor.h>
#include <gtsam/navigation/GPSFactor.h>
#include <gtsam/navigation/ImuFactor.h>
#include <gtsam/nonlinear/ISAM2.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/expressions.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/slam/dataset.h>
#include <gtsam/slam/expressions.h>
#include <gtsam/nonlinear/ExpressionFactorGraph.h>
#include <gtsam/geometry/Pose2.h>

#include <cassert>
#include <cstring>
#include <fstream>
#include <iostream>
#include <functional>
#include <deque>

#include "filter.h"

using namespace gtsam;
using namespace std;

#define BUFFER_SIZE 2
#define NODE_NUM    12
#define VEDIO_FRAME_SPEED   60
#define DT2 (VEDIO_FRAME_SPEED * VEDIO_FRAME_SPEED)
#define SCORE_NODE  0.5

static vector<Rot3> t_pose_rot3(NODE_NUM);
static Rot3 Rgi_prime;
static vector<Rot3> R_gamma(NODE_NUM, Rot3::Identity());
static vector<Expression<Rot3>> theta, gamma;
static vector<Rot3> imu_bais(12, Rot3::Identity());
static Expression<Point3> root_position(Symbol('r'));
static Point3 root_position_value = Point3(0, 0, 3);
static vector<noiseModel::Diagonal::shared_ptr> covariance(NODE_NUM);
static vector<int> joint_parents = {-1, 0, 1, 1, 2, 3, 4, 5, 0, 0, 8, 9};
static vector<Vector3> upper_limit(NODE_NUM), lower_limit(NODE_NUM);
static uint64_t current_count = 0;
static Vector3 global_g(0, 0, 9.81);
static deque<vector<Point3>> bone_postion_buffer;
static deque<vector<Rot3>> rotation_buffer;
static bool buffer_tag = false;
static vector<Vector3> lever_arm = {
    Vector3(0.000001 - 0.003123, 0.10923 - (-0.012037), (-0.28571) - (-0.35141)),
    Vector3(0.000003 - (-0.001522), -0.11128 - (-0.006926), -0.046152 - (-0.057428)),
    Vector3(0.36655 - 0.16408, 0.07428 - 0.015756, 0.077047 - 0.085243),
    Vector3(-0.36655 - (-0.15179), 0.074819 - 0.019143, 0.077047 - 0.080435),
    Vector3(0.61431 - 0.4182, 0.071717 - 0.058214, 0.065621 - 0.013093),
    Vector3(-0.63442 - (-0.42294), 0.068416 - 0.04561, 0.064987 - 0.043942),
    Vector3(0.72262 - 0.67019, 0.060991 - 0.060687, 0.053236 - 0.036314),
    Vector3(-0.67221 - (-0.72262), 0.060935 - 0.060992, 0.03941 - 0.053235),
    Vector3(0.15425 - 0.061313, 0.016086 - 0.013965, -0.75066 - (-0.44417)),
    Vector3(-0.14625 - (-0.060144), -0.014388 - 0.009214, -0.75913 - (-0.45532)),
    Vector3(0.096918 - 0.11601, 0.024403 - 0.023361, -1.1584 - (-0.82292)),
    Vector3(-0.096915 - (-0.10435), 0.024403 - 0.026038, -1.1584 - (-0.8177))
};
static vector<Vector3> bone_vector = {
    Vector3(0, 0, 0),
    Vector3(-0.001522 - 0.003123, -0.006926 - (-0.012037), -0.057428 - (-0.351407)),
    Vector3(0.164081 - (-0.001522), 0.015756 - (-0.006926), 0.085243 - (-0.057428)),
    Vector3(-0.151795 - (-0.001522), 0.019143 - (-0.006926), 0.080435 - (-0.057428)),
    Vector3(0.418204 - 0.164081, 0.058214 - 0.015756, 0.013093 - 0.085243),
    Vector3(-0.422944 - (-0.151795), 0.04561 - 0.019143, 0.043942 - 0.080435),
    Vector3(0.670191 - 0.418204, 0.060687 - 0.058214, 0.036314 - 0.013093),
    Vector3(-0.672212 - (-0.422944), 0.060935 - 0.04561, 0.03941 - 0.043942),
    Vector3(0.061313 - 0.003123, 0.013965 - (-0.012037), -0.444171 - (-0.351407)),
    Vector3(-0.060144 - 0.003123, 0.009214 - (-0.012037), -0.455316 - (-0.351407)),
    Vector3(0.116008 - 0.061313, 0.023361 - 0.013965, -0.822925 - (-0.444171)),
    Vector3(-0.104354 - (-0.060144), 0.026038 - (0.009214), -0.817695 - (-0.455316))
};
static auto K = gtsam::make_shared<Cal3_S2>(1.0, 1.0, 0, 1.0, 1.0);
static const Pose3 M(Pose3::Identity());
static PinholeCamera<Cal3_S2> camera(M, *K);
static Expression<PinholeCamera<Cal3_S2>> camera_expr(camera);

void init_tpose(float quat[NODE_NUM][4])
{
    //1.imu's quat to SO(3)
    for (int i = 0; i < NODE_NUM; ++i) {
        Rot3 rota = Rot3::Quaternion(quat[i][0], quat[i][1], quat[i][2], quat[i][3]);
        t_pose_rot3[i] = rota;
        theta.emplace_back(Expression<Rot3>(Symbol('j', i)));
        gamma.emplace_back(Expression<Rot3>(Symbol('b', i)));
    }
    //Rbs = (Rgb(theat0))^-1 * Rgi * Ris(0)
    //Ris(0) is the IMU reading in first frame, here is t_pose_rot3
    //Set rgb and rgi is I, so Rbs == Ris(0)

    //Rgb^ = Rgi' * Ri'i(gamma) * Ris * Rbs^-1
    //Set Rgi'
    Vector3 local_forwad(0, 1, 0);
    Vector3 inertial_forward = t_pose_rot3[0].rotate(local_forwad);
    double yaw_calc = atan2(inertial_forward.y(), inertial_forward.x());
    Rot3 r_yaw_align = Rot3::Yaw(-yaw_calc);

    Rgi_prime = r_yaw_align;

    covariance[0] = noiseModel::Diagonal::Sigmas(Vector3(1.5039548406731926, 1.3604991226001177, 1.3903372687663433));
    covariance[1] = noiseModel::Diagonal::Sigmas(Vector3(0.14239438988003683, 0.06630053091618329, 0.0687266885688208));
    covariance[2] = noiseModel::Diagonal::Sigmas(Vector3(0.19858312909601475, 0.24931291143087014, 0.3476067819453756));
    covariance[3] = noiseModel::Diagonal::Sigmas(Vector3(0.24724707214743108, 0.25532520741517767, 0.36446417269409437));
    covariance[4] = noiseModel::Diagonal::Sigmas(Vector3(0.30056719783021113, 0.5888782078883058, 0.2488598225092799));
    covariance[5] = noiseModel::Diagonal::Sigmas(Vector3(0.29475092345901294, 0.6029379638658562, 0.2607840857852203));
    covariance[6] = noiseModel::Diagonal::Sigmas(Vector3(0.31296337418518294, 0.15291605186657653, 0.31806588380726303));
    covariance[7] = noiseModel::Diagonal::Sigmas(Vector3(0.31269783712934207, 0.14621038392492183, 0.3264431255801909));
    covariance[8] = noiseModel::Diagonal::Sigmas(Vector3(0.34202164100137467, 0.10071092222917752, 0.14622032215567957));
    covariance[9] = noiseModel::Diagonal::Sigmas(Vector3(0.3391136973512318, 0.09152914507483235, 0.14035045249473455));
    covariance[10] = noiseModel::Diagonal::Sigmas(Vector3(0.4474578497070993, 0.18532047439932334, 0.09596088441816034));
    covariance[11] = noiseModel::Diagonal::Sigmas(Vector3(0.4474059796179263, 0.1527556887968425, 0.08514769431184203));

// Node 0: Pelvis (通常根节点不限位，这里是数据集统计值)
    lower_limit[0] = (Vector3() << -4.7917, -2.5428, -2.6983).finished();
    upper_limit[0] = (Vector3() <<  2.0997,  2.4989,  2.6678).finished();

    // Node 1: Chest (Spine3)
    lower_limit[1] = (Vector3() << -0.5047, -0.5161, -0.3273).finished();
    upper_limit[1] = (Vector3() <<  0.6172,  0.5575,  0.2012).finished();

    // Node 2: L_UpperArm (左大臂)
    lower_limit[2] = (Vector3() << -0.5102, -0.9710, -1.2947).finished();
    upper_limit[2] = (Vector3() <<  0.4779,  0.2644,  0.2535).finished();

    // Node 3: R_UpperArm (右大臂)
    lower_limit[3] = (Vector3() << -0.6475, -0.2915, -0.2705).finished();
    upper_limit[3] = (Vector3() <<  0.4803,  0.9680,  1.2840).finished();

    // Node 4: L_LowerArm (左小臂 - 重点看 Y 轴弯曲)
    lower_limit[4] = (Vector3() << -0.8455, -2.2169, -0.3266).finished();
    upper_limit[4] = (Vector3() <<  0.7431,  0.0074,  1.0212).finished();

    // Node 5: R_LowerArm (右小臂)
    lower_limit[5] = (Vector3() << -0.8739, -0.0061, -1.1210).finished();
    upper_limit[5] = (Vector3() <<  0.6928,  2.2361,  0.3732).finished();

    // Node 6: L_Hand (左手掌)
    lower_limit[6] = (Vector3() << -1.1142, -0.6004, -0.7918).finished();
    upper_limit[6] = (Vector3() <<  0.5100,  0.1470,  0.9441).finished();

    // Node 7: R_Hand (右手掌)
    lower_limit[7] = (Vector3() << -1.0434, -0.2143, -0.8400).finished();
    upper_limit[7] = (Vector3() <<  0.5970,  0.5433,  0.9762).finished();

    // Node 8: L_UpperLeg (左大腿)
    lower_limit[8] = (Vector3() << -1.3043, -0.2475, -0.1686).finished();
    upper_limit[8] = (Vector3() <<  0.3840,  0.2680,  0.5957).finished();

    // Node 9: R_UpperLeg (右大腿)
    lower_limit[9] = (Vector3() << -1.3352, -0.2703, -0.5500).finished();
    upper_limit[9] = (Vector3() <<  0.3224,  0.2096,  0.1706).finished();

    // Node 10: L_LowerLeg (左小腿 - 重点看 X 轴弯曲)
    lower_limit[10] = (Vector3() << -0.1470, -0.5152, -0.4308).finished();
    upper_limit[10] = (Vector3() <<  2.0146,  0.4132,  0.0804).finished();

    // Node 11: R_LowerLeg (右小腿)
    lower_limit[11] = (Vector3() << -0.0677, -0.4284, -0.1207).finished();
    upper_limit[11] = (Vector3() <<  2.0768,  0.3563,  0.3238).finished();

}

Vector3 joint_limit_error(const Rot3& R, const Vector3& lower, const Vector3& upper, OptionalJacobian<3, 3> H = {})
{
    Matrix33 Jlog = Matrix33::Zero(), Dlog = Matrix33::Zero();
    Vector3 logmap = Rot3::Logmap(R, H ? &Jlog : nullptr);
    Vector3 error;

    for (int i = 0; i < 3; ++i) {
        if (logmap[i] < lower[i]) {
            error[i] = logmap[i] - lower[i];
            Dlog(i, i) = 1.0;
        } else if (logmap[i] > upper[i]) {
            error[i] = logmap[i] - upper[i];
            Dlog(i, i) = 1.0;
        } else {
            error[i] = 0;
        }
    }

    if (H) {
        *H = Dlog * Jlog;
    }

    return error;
}

void updata_only_imu(const float imu_data[NODE_NUM][4], float get_pose[NODE_NUM][4])
{
    Values initial_value;
    ExpressionFactorGraph graph;
    vector<Rot3> global_rotation(NODE_NUM);
    auto limit_noise = noiseModel::Isotropic::Sigma(3, 0.01);
    auto model = noiseModel::Diagonal::Sigmas(Vector3(0.005, 0.005, 0.01));
    vector<Rot3> Ris(NODE_NUM);

    for (int i = 0; i < NODE_NUM; ++i) {
        Ris[i] = Rot3::Quaternion(imu_data[i][0], imu_data[i][1], imu_data[i][2], imu_data[i][3]);
        global_rotation[i] = Rgi_prime * R_gamma[i] * Ris[i] * t_pose_rot3[i].inverse();
        initial_value.insert(Symbol('j', i), global_rotation[i]);
    }

    for (int i = 0; i < NODE_NUM; ++i) {
        graph.addExpressionFactor(theta[i], global_rotation[i], model);
        if (i == 0) {
            //graph.addExpressionFactor(theta[i], Rot3::Identity(), covariance[i]);
        }
        else {
            graph.addExpressionFactor(between(theta[joint_parents[i]], theta[i]), Rot3::Identity(), covariance[i]);
            Expression<Vector3>::UnaryFunction<Rot3>::type bound_func = bind(&joint_limit_error, placeholders::_1, lower_limit[i], upper_limit[i], placeholders::_2);
            Expression<Vector3> limit_expr(bound_func, between(theta[joint_parents[i]], theta[i]));
            graph.addExpressionFactor(limit_expr, (Vector3)Vector3::Zero(), limit_noise);
        }
    }

    Values result = LevenbergMarquardtOptimizer(graph, initial_value).optimize();

    for (int i = 0; i < NODE_NUM; ++i) {
        Rot3 rot = result.at<Rot3>(Symbol('j', i));
        auto q = rot.toQuaternion();

        get_pose[i][0] = q.w();
        get_pose[i][1] = q.x();
        get_pose[i][2] = q.y();
        get_pose[i][3] = q.z();
    }

}

void updata_with_vedio(float imu_data[NODE_NUM][4], float accel[NODE_NUM][3], float position_2D[NODE_NUM][2], float score[NODE_NUM], float get_pose[NODE_NUM][4])
{
    Values initial_values;
    ExpressionFactorGraph graph;
    vector<Rot3> Ris(NODE_NUM);
    auto model = noiseModel::Diagonal::Sigmas(Vector3(0.05, 0.05, 0.2));
    auto accel_noise = noiseModel::Isotropic::Sigma(3, 1.0);
    auto limit_noise = noiseModel::Isotropic::Sigma(3, 0.01);
    vector<Vector3> accel_v(NODE_NUM);
    vector<Point2> point_2D(NODE_NUM);
    vector<Expression<gtsam::Point3>> node_position;
    vector<Expression<Rot3>> global_rotation;

    //wait to fix
    for (int i = 0; i < NODE_NUM; ++i) {
        Ris[i] = Rot3::Quaternion(imu_data[i][0], imu_data[i][1], imu_data[i][2], imu_data[i][3]);
        accel_v[i] = Vector3(accel[i][0], accel[i][1], accel[i][2]);
        point_2D[i] = Point2(position_2D[i][0], position_2D[i][1]);
        if (joint_parents[i] == -1) {
            node_position.push_back(Point3_(root_position));
        } else {
            node_position.push_back(node_position[joint_parents[i]] + rotate(theta[joint_parents[i]], Point3_(bone_vector[i])));
        }
        initial_values.insert(Symbol('j', i), Ris[i]);
        initial_values.insert(Symbol('b', i), imu_bais[i]);
    }
    initial_values.insert(Symbol('r'), root_position_value);

    for (int i = 0; i < NODE_NUM; ++i) {
        global_rotation.push_back(Rot3_(Rgi_prime) * gamma[i] * Rot3_(Ris[i]) * Rot3_(t_pose_rot3[i].inverse()));
        graph.addExpressionFactor(between(theta[i], global_rotation[i]), Rot3::Identity(), model);

        if (buffer_tag) {
            Rot3_ rotation = Rot3_(Rgi_prime) * gamma[i] * Rot3_(Ris[i]);
            Expression<Point3> prediction_accel = rotate(rotation, Vector3_(accel_v[i])) - Vector3_(global_g);

            Point3 p_prev = bone_postion_buffer[0][i] + rotation_buffer[0][i].rotate(lever_arm[i]);
            Point3 p_pprev = bone_postion_buffer[1][i] + rotation_buffer[1][i].rotate(lever_arm[i]);
            Point3_ p_t = node_position[i] + rotate(theta[i], Point3_(lever_arm[i]));
            Point3_ const_term = Point3_((p_pprev - p_prev * (2.0)) * (double)DT2);
            Expression<Point3> a_hat = Point3_(const_term) + (double)DT2 * p_t;
            graph.addExpressionFactor(between(a_hat, prediction_accel), Point3(0, 0, 0), accel_noise);
        }

        if (score[i] > SCORE_NODE) {
            Expression<Point2> project_2d = project2(camera_expr, node_position[i]);
            auto img_noise = noiseModel::Isotropic::Sigma(2, 1.0 / (score[i] + 1e-6));
            auto robust_img_noise = noiseModel::Robust::Create(noiseModel::mEstimator::Huber::Create(1.0), img_noise);
            graph.addExpressionFactor(project_2d, point_2D[i], robust_img_noise);
        }

        if (i == 0) {
            //graph.addExpressionFactor(theta[i], Rot3::Identity(), covariance[i]);
        }
        else {
            graph.addExpressionFactor(between(theta[joint_parents[i]], theta[i]), Rot3::Identity(), covariance[i]);
            Expression<Vector3>::UnaryFunction<Rot3>::type bound_func = bind(&joint_limit_error, placeholders::_1, lower_limit[i], upper_limit[i], placeholders::_2);
            Expression<Vector3> limit_expr(bound_func, between(theta[joint_parents[i]], theta[i]));
            graph.addExpressionFactor(limit_expr, (Vector3)Vector3::Zero(), limit_noise);
        }
    }

    vector<Rot3> temp_rota_buffer(12);
    vector<Point3> temp_posi_buffer(12);
    Values result = LevenbergMarquardtOptimizer(graph, initial_values).optimize();
    root_position_value = result.at<Point3>(Symbol('r'));
    for (int i = 0; i < NODE_NUM; ++i) {
        temp_rota_buffer[i] = result.at<Rot3>(Symbol('j', i));
        auto q_pose = temp_rota_buffer[i].toQuaternion();
        imu_bais[i] = result.at<Rot3>(Symbol('b', i));
        get_pose[i][0] = q_pose.w();
        get_pose[i][1] = q_pose.x();
        get_pose[i][2] = q_pose.y();
        get_pose[i][3] = q_pose.z();
    }

    if (buffer_tag == true) {
        rotation_buffer.pop_front();
        bone_postion_buffer.pop_front();
    }


    for (int i = 0; i < 12; ++i) {
        if (joint_parents[i] == -1) {
            temp_posi_buffer[i] = root_position_value;
        } else {
            temp_posi_buffer[i] = temp_posi_buffer[joint_parents[i]] + temp_rota_buffer[i].rotate(bone_vector[i]);
        }
    }
    rotation_buffer.push_back(temp_rota_buffer);
    bone_postion_buffer.push_back(temp_posi_buffer);
}

void test_func()
{
    cout << "test ok" << endl;
}
