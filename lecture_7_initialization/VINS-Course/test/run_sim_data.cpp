//
// Created by meng on 2021/10/29.
//
#include "System.h"
#include "generate_sim_data/imu.h"
#include "generate_sim_data/utilities.h"
#include "global_defination.h"

#include <sys/stat.h>

using Point = Eigen::Vector4d;
using Points = std::vector<Point, Eigen::aligned_allocator<Point> >;
using Line = std::pair<Eigen::Vector4d, Eigen::Vector4d>;
using Lines = std::vector<Line, Eigen::aligned_allocator<Line> >;

using namespace Eigen;
using namespace std;

const int nDelayTimes = 2;
string sConfig_path = "../config";
std::shared_ptr<System> pSystem;

/*!
 * 读取文件中的点，然后生成空间点和点之间的连线
 * @param points
 * @param lines
 */
void CreatePointsLines(Points &points, Lines &lines) {
    std::ifstream f;
    f.open(WORK_SPACE_PATH
           + "/house_model/house.txt");

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
            for (const auto &pt: points) {
                if (pt == pt0) {
                    isHistoryPoint = true;
                }
            }
            if (!isHistoryPoint)
                points.push_back(pt0);

            isHistoryPoint = false;
            for (const auto &pt: points) {
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

void GenerateData() {
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
        for (auto &point: points) {
            Eigen::Vector4d pw = point;// 最后一位存着feature id
            pw[3] = 1;//改成齐次坐标最后一位
            Eigen::Vector4d pc1 = Twc.inverse() * pw;// T_wc.inverse() * Pw  -- > point in cam frame

            if (pc1(2) < 0) continue;// z必须大于０,在摄像机坐标系前方

            Eigen::Vector2d obs(pc1(0) / pc1(2), pc1(1) / pc1(2));
            // if( (obs(0)*460 + 255) < params.image_h && ( obs(0) * 460 + 255) > 0 &&
            // (obs(1)*460 + 255) > 0 && ( obs(1)* 460 + 255) < params.image_w )
            {
                points_cam.push_back(point);
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
        for (const auto &line_pt: lines) {
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
}

void PubImuData() {
    string sImu_data_file = WORK_SPACE_PATH + "/bin/imu_pose_noise.txt";//带噪声的IMU数据的路径
    cout << "1 PubImuData start sImu_data_filea: " << sImu_data_file << endl;
    ifstream fsImu;//文件流对象
    fsImu.open(sImu_data_file.c_str());
    if (!fsImu.is_open()) {
        cerr << "Failed to open imu file! " << sImu_data_file << endl;
        return;
    }

    std::string sImu_line;
    double dStampNSec = 0.0;//时间戳
    double tmp;
    Vector3d vAcc;//加速度
    Vector3d vGyr;
    while (std::getline(fsImu, sImu_line) && !sImu_line.empty()) // read imu data sImu_line获得每行的文件流
    {
        // timestamp (1)，imu quaternion(4)，imu position(3)，imu gyro(3)，imu acc(3)
        std::istringstream ssImuData(sImu_line);//ssImuData得到每行文件的内容
        ssImuData >> dStampNSec;//时间戳
        //利用循环跳过imu quaternion(4)，imu position(3)
        for (int i = 0; i < 7; i++)
            ssImuData >> tmp;
        ssImuData >> vGyr.x() >> vGyr.y() >> vGyr.z() >> vAcc.x() >> vAcc.y() >> vAcc.z();
        // 时间单位为 s
        pSystem->PubImuData(dStampNSec, vGyr, vAcc);//PubImuData不需要改变 用来将不同时刻下的IMU数据放进VINS系统
        usleep(5000 * nDelayTimes);
    }
    fsImu.close();
}

//把相机的特征点放进VINS---里面用到的PubSimImageData函数要在system.cpp写入
void PubImageData() {
    string sImage_file = WORK_SPACE_PATH + "/bin/cam_pose.txt";  //相机相关数据的路径
    //
    //包含时间戳的文件

    cout << "1 PubImageData start sImage_file: " << sImage_file << endl;

    ifstream fsImage;//文件流对象
    fsImage.open(sImage_file.c_str());
    if (!fsImage.is_open()) {
        cerr << "Failed to open image file! " << sImage_file << endl;
        return;
    }

    std::string sImage_line;
    double dStampNSec;
    string sImgFileName;
    int n = 0;

    // cv::namedWindow("SOURCE IMAGE", CV_WINDOW_AUTOSIZE);
    //这个循环是遍历所有的相机
    while (std::getline(fsImage, sImage_line) && !sImage_line.empty())//sImage_line是cam_pose每行的数据流
    {
        std::istringstream ssImgData(sImage_line);//是cam_pose每行的内容
        ssImgData >> dStampNSec;   //读入时间戳
        cout << "cam time: " << fixed << dStampNSec << endl;
        // all_points_ 文件存储的是house模型的线特征，每行4个数，对应该线两端点在归一化平面的坐标
        //all_points_  文件每行的内容是 x, y, z, 1, u, v  这里的u v是归一化下的x ,y 不是像素坐标
        //在函数PubSimImageData中会算出具体特征点的像素坐标
        string all_points_file_name =
                "/home/nnz/data/vio/bin/keyframe/all_points_" + to_string(n) + ".txt";  //第n个相机对应的观测数据的文件名
        cout << "points_file: " << all_points_file_name << endl;
        vector<cv::Point2f> FeaturePoints;//容器FeaturePoints存放一个相机的特征点(归一化坐标)
        std::ifstream f;
        f.open(all_points_file_name);

        //这个循环是遍历每个相机的特征点信息
        // file content in each line: x, y, z, 1, u, v
        //经过这个循环把all_points_的特征点都放在FeaturePoints了
        while (!f.eof()) {
            std::string s;
            std::getline(f, s);//得到all_points_的文件流s
            // 一行两个点连成线，获取每行点判断一下是否之前获取过
            if (!s.empty()) {
                std::stringstream ss;//
                ss << s;//ss得到每行的内容

                double tmp;//跳过  x y z 1
                for (int i = 0; i < 4; i++)
                    ss >> tmp;

                float px, py;
                ss >> px;
                ss >> py;
                cv::Point2f pt(px, py);//归一化坐标

                FeaturePoints.push_back(pt);
            }
        }

//        cout << "All points:" << endl;
//        for(auto point : FeaturePoints){
//            cout << point << " ";
//        }
//        cout << endl;


        pSystem->PubSimImageData(dStampNSec, FeaturePoints);//把每一个图片的特征点 放进VINS系统里

        usleep(50000 * nDelayTimes);
        n++;
    }
    fsImage.close();
}

int main(int argc, char **argv) {
    GenerateData();

//    if (argc != 2) {
//        cerr << "./run_sim_data PATH_TO_CONFIG/config \n"
//             << "For example: ./run_sim_data ../config/" << endl;
//        return -1;
//    }
//    sConfig_path = argv[2];

    pSystem.reset(new System(sConfig_path));

    std::thread thd_BackEnd(&System::ProcessBackEnd, pSystem);

    // sleep(5);
    std::thread thd_PubImuData(PubImuData);

    std::thread thd_PubImageData(PubImageData);

#ifdef __linux__
    std::thread thd_Draw(&System::Draw, pSystem);
#elif __APPLE__
    DrawIMGandGLinMainThrd();
#endif

    thd_PubImuData.join();
    thd_PubImageData.join();

    // thd_BackEnd.join();
    // thd_Draw.join();

    cout << "main end... see you ..." << endl;
    return 0;
}