//
// Created by hyj on 17-6-22.
//
#include <fstream>
#include <sys/stat.h>
#include "../src/imu.h"
#include "../src/utilities.h"
#include "global_defination.h"

using Point = Eigen::Vector4d;
using Points = std::vector<Point, Eigen::aligned_allocator<Point> >;
using Line = std::pair<Eigen::Vector4d, Eigen::Vector4d>;
using Lines = std::vector<Line, Eigen::aligned_allocator<Line> >;

/*!
 * 读取文件中的点，然后生成空间点和点之间的连线
 * @param points
 * @param lines
 */
void CreatePointsLines(Points &points, Lines &lines) {
    std::ifstream f;
    f.open(WORK_SPACE_PATH + "/house_model/house.txt");

    while (!f.eof()) {
        std::string s;
        std::getline(f, s);
        if (!s.empty()) {
            std::stringstream ss;
            ss << s;
            double x, y, z;
            ss >> x;
            ss >> y;
            ss >> z;
            Eigen::Vector4d pt0(x, y, z, 1);
            ss >> x;
            ss >> y;
            ss >> z;
            Eigen::Vector4d pt1(x, y, z, 1);

            bool isHistoryPoint = false;
            for (unsigned int i = 0; i < points.size(); ++i) {
                Eigen::Vector4d pt = points[i];
                if (pt == pt0) {
                    isHistoryPoint = true;
                }
            }
            if (!isHistoryPoint)
                points.push_back(pt0);

            isHistoryPoint = false;
            for (unsigned int i = 0; i < points.size(); ++i) {
                Eigen::Vector4d pt = points[i];
                if (pt == pt1) {
                    isHistoryPoint = true;
                }
            }
            if (!isHistoryPoint)
                points.push_back(pt1);

            // pt0 = Twl * pt0;
            // pt1 = Twl * pt1;
            lines.emplace_back(pt0, pt1);   // lines
        }
    }

    // create more 3d points
    unsigned int n = points.size();
    for (unsigned int j = 0; j < n; ++j) {
        Eigen::Vector4d p = points[j] + Eigen::Vector4d(0.5, 0.5, -0.5, 0);
        points.push_back(p);
    }

    // save points
    SavePoints(WORK_SPACE_PATH + "bin/all_points.txt", points);
}

int main() {
    // 建立keyframe文件夹
    mkdir("keyframe", 0777);

    // 生成3d points
    Points points;
    Lines lines;
    CreatePointsLines(points, lines);

    // IMU model
    Param params;
    IMU imuGen(params);

    // create imu data
    // imu pose gyro acc
    std::vector<MotionData> imu_data;
    std::vector<MotionData> imu_data_noise;
    for (double t = params.t_start; t < params.t_end;) {
        MotionData data = imuGen.MotionModel(t);
        imu_data.push_back(data);

        // add imu noise
        MotionData data_noise = data;
        imuGen.addIMUnoise(data_noise);
        imu_data_noise.push_back(data_noise);

        t += 1.0 / params.imu_frequency;
    }
    imuGen.init_velocity_ = imu_data.at(0).imu_velocity;
    imuGen.init_twb_ = imu_data.at(0).twb;
    imuGen.init_Rwb_ = imu_data.at(0).Rwb;
    SavePose(WORK_SPACE_PATH + "/bin/imu_pose.txt", imu_data);
    SavePose(WORK_SPACE_PATH + "/bin/imu_pose_noise.txt", imu_data_noise);

    //积分IMU的原始数据，获得轨迹
    imuGen.testImu(WORK_SPACE_PATH + "/bin/imu_pose.txt", "imu_int_pose.txt");
    //积分带噪声的IMU数据，获得轨迹
    imuGen.testImu(WORK_SPACE_PATH + "/bin/imu_pose_noise.txt",
                   WORK_SPACE_PATH + "/bin/imu_int_pose_noise.txt");

    // cam pose
    std::vector<MotionData> cam_data;
    for (double t = params.t_start; t < params.t_end;) {

        MotionData imu = imuGen.MotionModel(t);//in Body frame
        MotionData cam;

        cam.timestamp = imu.timestamp;
        cam.Rwb = imu.Rwb * params.R_bc;// cam frame in world frame
        cam.twb = imu.twb + imu.Rwb * params.t_bc;// Tcw = Twb * Tbc ,  t = Rwb * tbc + twb

        cam_data.push_back(cam);
        t += 1.0 / params.cam_frequency;
    }
    SavePose(WORK_SPACE_PATH + "/bin/cam_pose.txt", cam_data);
    SavePoseAsTUM(WORK_SPACE_PATH + "/bin/cam_pose_tum.txt", cam_data);

    // points obs in image
    for (unsigned int n = 0; n < cam_data.size(); ++n) {
        MotionData data = cam_data[n];
        Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
        Twc.block(0, 0, 3, 3) = data.Rwb;//这里虽然是Rwb, 但是实际是camera->world
        Twc.block(0, 3, 3, 1) = data.twb;

        // 遍历所有的特征点，看哪些特征点在视野里
        std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > points_cam;    // ３维点在当前cam视野里
        std::vector<Eigen::Vector2d, Eigen::aligned_allocator<Eigen::Vector2d> > features_cam;  // 对应的２维图像坐标
        for (unsigned int i = 0; i < points.size(); ++i) {
            Eigen::Vector4d pw = points[i];// 最后一位存着feature id
            pw[3] = 1;//改成齐次坐标最后一位
            Eigen::Vector4d pc1 = Twc.inverse() * pw;// T_wc.inverse() * Pw  -- > point in cam frame

            if (pc1(2) < 0) continue;// z必须大于０,在摄像机坐标系前方

            Eigen::Vector2d obs(pc1(0) / pc1(2), pc1(1) / pc1(2));
            // if( (obs(0)*460 + 255) < params.image_h && ( obs(0) * 460 + 255) > 0 &&
            // (obs(1)*460 + 255) > 0 && ( obs(1)* 460 + 255) < params.image_w )
            {
                points_cam.push_back(points[i]);
                features_cam.push_back(obs);
            }
        }

        // save points
        std::stringstream filename1;
        filename1 << WORK_SPACE_PATH + "/bin/keyframe/all_points_" << n << ".txt";
        SaveFeatures(filename1.str(), points_cam, features_cam);
    }

    // lines obs in image
    for (unsigned int n = 0; n < cam_data.size(); ++n) {
        MotionData data = cam_data[n];
        Eigen::Matrix4d Twc = Eigen::Matrix4d::Identity();
        Twc.block(0, 0, 3, 3) = data.Rwb;
        Twc.block(0, 3, 3, 1) = data.twb;

        // 遍历所有的特征点，看哪些特征点在视野里
        // std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > points_cam;    // ３维点在当前cam视野里
        std::vector<Eigen::Vector4d, Eigen::aligned_allocator<Eigen::Vector4d> > features_cam;  // 对应的２维图像坐标
        for (unsigned int i = 0; i < lines.size(); ++i) {
            Line line_pt = lines[i];

            Eigen::Vector4d pc1 = Twc.inverse() * line_pt.first;// T_wc.inverse() * Pw  -- > point in cam frame
            Eigen::Vector4d pc2 = Twc.inverse() * line_pt.second;// T_wc.inverse() * Pw  -- > point in cam frame

            if (pc1(2) < 0 || pc2(2) < 0) continue;// z必须大于０,在摄像机坐标系前方

            Eigen::Vector4d obs(pc1(0) / pc1(2), pc1(1) / pc1(2),
                                pc2(0) / pc2(2), pc2(1) / pc2(2));
            //if(obs(0) < params.image_h && obs(0) > 0 && obs(1)> 0 && obs(1) < params.image_w)
            {
                features_cam.push_back(obs);
            }
        }

        // save points
        std::stringstream filename1;
        filename1 << WORK_SPACE_PATH + "/bin/keyframe/all_lines_" << n << ".txt";
        SaveLines(filename1.str(), features_cam);
    }

    return 0;
}