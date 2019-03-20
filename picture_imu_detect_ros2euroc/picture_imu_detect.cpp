#include <cstdio>
#include <stdio.h>
#include <queue>
#include <map>
#include <thread>
#include <mutex>
#include <sys/stat.h>
#include <sys/types.h>
#include <dirent.h>
#include <unistd.h>
#include <condition_variable>
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <std_msgs/Header.h>
#include <std_msgs/Float32.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>        //sensor_msgs::ImuConstPtr
#include <opencv2/opencv.hpp>
using namespace std;

#define IMU_TOPIC "/camera/imu/data_raw"
#define IMAGE_TOPIC "/camera/color/image_raw"

void imu_callback(const sensor_msgs::ImuConstPtr &imu_msg)
{
//    m_buf.lock();
//    imu_buf.push(imu_msg);
//    m_buf.unlock();
//    con.notify_one();
//
//    {
//        std::lock_guard<std::mutex> lg(m_state);
//        predict(imu_msg);
//        std_msgs::Header header = imu_msg->header;
//        header.frame_id = "world";
//        if (estimator.solver_flag == Estimator::SolverFlag::NON_LINEAR)
//            pubLatestOdometry(tmp_P, tmp_Q, tmp_V, header);
//    }
    long long int time = imu_msg->header.stamp.toNSec();
    double dx = imu_msg->linear_acceleration.x;
    double dy = imu_msg->linear_acceleration.y;
    double dz = imu_msg->linear_acceleration.z;

    double rx = imu_msg->angular_velocity.x;
    double ry = imu_msg->angular_velocity.y;
    double rz = imu_msg->angular_velocity.z;

    FILE* imu_csv_out = fopen("mav0/imu0/data.csv", "a+");
    fprintf(imu_csv_out, "%lld,%.6f,%.6f,%.6f,%.6f,%.6f,%.6f\n", time, dx, dy, dz, rx, ry, rz);
    fclose(imu_csv_out);
}

void raw_image_callback(const sensor_msgs::ImageConstPtr &img_msg)
{
    cv_bridge::CvImagePtr img_ptr = cv_bridge::toCvCopy(img_msg, sensor_msgs::image_encodings::MONO8);
    //image_pool[img_msg->header.stamp.toNSec()] = img_ptr->image;
    long long int time_stamp_s = img_msg->header.stamp.toNSec();
    stringstream s;
    s << time_stamp_s;
    std::string picture_name = s.str() + ".png";
    printf("%lld,%s\n", time_stamp_s, picture_name.c_str());
    FILE* data_csv_out = fopen("mav0/cam0/data.csv", "a+");
    fprintf(data_csv_out, "%lld,%s\n", time_stamp_s, picture_name.c_str());
    fclose(data_csv_out);

    cv::Mat image = img_ptr->image;
    cv::imshow("image", image);
    std::string picture_path = "mav0/cam0/data/" + picture_name;
    cv::imwrite(picture_path.c_str(), image);
    cv::waitKey(5);

}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "picture_imu_detect");
    ros::NodeHandle n("~");
    ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Info);
#ifdef EIGEN_DONT_PARALLELIZE
    ROS_DEBUG("EIGEN_DONT_PARALLELIZE");
#endif
    ROS_WARN("waiting for image and imu...");

    //判断目录是否存在，不存在创建mav0目录
    std::string mav0 = getcwd(NULL, 200);
    mav0 +=  + "/mav0";
    DIR *dp_mav0_path;
    dp_mav0_path = opendir(mav0.c_str());
    if(dp_mav0_path == NULL)
    {
        cout << "no mav0 path" << endl;
        if(0 == mkdir(mav0.c_str(), S_IRWXU | S_IRWXG | S_IRWXO))
        {
            cout << "creat mav0 path successful!" << endl;
        } else
        {
            cout << "creat mav0 path failed!" << endl;
            return -1;
        }
    }
    closedir(dp_mav0_path);

    std::string cam0 = getcwd(NULL, 200);
    cam0 +=  + "/mav0/cam0";
    DIR *dp_cam0_path;
    dp_cam0_path = opendir(cam0.c_str());
    if(dp_cam0_path == NULL)
    {
        cout << "no cam0 path" << endl;
        if(0 == mkdir(cam0.c_str(), S_IRWXU | S_IRWXG | S_IRWXO))
        {
            cout << "creat cam0 path successful!" << endl;
        } else
        {
            cout << "creat cam0 path failed!" << endl;
            return -1;
        }
    }
    closedir(dp_cam0_path);

    std::string imu0 = getcwd(NULL, 200);
    imu0 +=  + "/mav0/imu0";
    DIR *dp_imu0_path;
    dp_imu0_path = opendir(imu0.c_str());
    if(dp_imu0_path == NULL)
    {
        cout << "no imu0 path" << endl;
        if(0 == mkdir(imu0.c_str(), S_IRWXU | S_IRWXG | S_IRWXO))
        {
            cout << "creat imu0 path successful!" << endl;
        } else
        {
            cout << "creat imu0 path failed!" << endl;
            return -1;
        }
    }
    closedir(dp_imu0_path);

    std::string image_data = getcwd(NULL, 200);
    image_data +=  + "/mav0/cam0/data";
    DIR *dp_image_data_path;
    dp_image_data_path = opendir(image_data.c_str());
    if(dp_image_data_path == NULL)
    {
        cout << "no image_data path" << endl;
        if(0 == mkdir(image_data.c_str(), S_IRWXU | S_IRWXG | S_IRWXO))
        {
            cout << "creat image_data path successful!" << endl;
        } else
        {
            cout << "creat image_data path failed!" << endl;
            return -1;
        }
    }
    closedir(dp_image_data_path);

    FILE* data_csv_out = fopen("mav0/cam0/data.csv", "w+");
    fprintf(data_csv_out, "%s\n", "#timestamp [ns],filename");
    fclose(data_csv_out);

    FILE* imu_csv_out = fopen("mav0/imu0/data.csv", "w+");
    fprintf(imu_csv_out, "%s\n", "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]");
    fclose(imu_csv_out);

    ros::Subscriber sub_imu = n.subscribe(IMU_TOPIC, 2000, imu_callback, ros::TransportHints().tcpNoDelay());
    ros::Subscriber sub_raw_image = n.subscribe(IMAGE_TOPIC, 2000, raw_image_callback);

    ros::spin();

    return 0;
}