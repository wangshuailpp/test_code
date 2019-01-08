#include <iostream>
#include <vector>
#include <string>

#include "eigen3/Eigen/Core"
#include "eigen3/Eigen/Geometry"
#include <opencv2/core/mat.hpp>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/core/cvstd.hpp>
#include <opencv2/core/types_c.h>
#include <opencv2/imgproc.hpp>
#include <opencv2/core/cvstd.inl.hpp>
#include <opencv2/core/eigen.hpp>


#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/opencv.hpp>

using namespace cv;



int main(int argc, char** argv) {

    if(argc < 2)
    {
        std::cerr << "please  input './test your_image_path'!" << std::endl;
        return -1;
    }

    cv::Mat pre_iamge;
    cv::Mat pre_gray;
    bool first_image = true;
    std::string image_path = argv[1];
    std::vector<cv::KeyPoint> cv_key_points;
    std::vector<cv::Point2f> points1;
    std::vector<cv::Point2f> points2;
    std::vector<uchar> status;
    std::vector<float> err;

    for(size_t i = 0; i < 30; ++i)
    {
        std::stringstream num1;
        num1 << i;
        std::string image_str1 = image_path + "/" + num1.str() + ".png";
        std::cout << "Debug: iamge1: " << image_str1 << std::endl;
        cv::Mat cur_image = cv::imread(image_str1);
        cv::imshow("current image", cur_image);


        //remove distort
        cv::Mat un_dis_image;
        cv::Mat cv_K(3, 3, CV_32F, cv::Scalar::all(0));
        cv::Mat cv_coeffs(1, 4, CV_32F, cv::Scalar::all(0));

        cv_K.at<float>(0, 0) = 458.654;
        cv_K.at<float>(1, 1) = 457.296;
        cv_K.at<float>(0, 2) = 367.215;
        cv_K.at<float>(1, 2) = 248.375;
        cv_K.at<float>(2, 2) = 1.0;

        cv_coeffs.at<float>(0) = -0.28340811;
        cv_coeffs.at<float>(1) = 0.07395907;
        cv_coeffs.at<float>(2) = 0.00019359;
        cv_coeffs.at<float>(3) = 0.00001762;

        cv::undistort(cur_image, un_dis_image, cv_K, cv_coeffs);
        cur_image = un_dis_image.clone();



        cv::Mat cur_gray;
        cv::cvtColor(cur_image, cur_gray, CV_RGB2GRAY);
        //when use this play bad!
//        static cv::Ptr<cv::CLAHE> clahe = cv::createCLAHE(6.0, cv::Size(8, 8 ));
//        clahe->apply(cur_gray, cur_gray);

        if(!first_image)
        {
            cv::cvtColor(pre_gray, pre_iamge, CV_GRAY2RGB);
            std::cout << "Debug: image channles: " << pre_gray.channels() << std::endl;
            //choose goodFeaturesToTrack or GFTTDetector
//            cv::goodFeaturesToTrack(pre_gray, points1, 300, 0.01, 10, cv::Mat());
            static cv::Ptr<cv::GFTTDetector> gftt = cv::GFTTDetector::create(1000, 1.0e-3, 20, 3, true);
            gftt->detect(pre_gray, cv_key_points);
            for(size_t i = 0; i < cv_key_points.size(); ++i)
            {
                points1.emplace_back(cv_key_points[i].pt.x, cv_key_points[i].pt.y);
            }
//            cv::calcOpticalFlowPyrLK(pre_iamge, cur_image, points1, points2, status, err);
            cv::calcOpticalFlowPyrLK(pre_gray, cur_gray, points1, points2, status, err);
            //play
            for(size_t i = 0; i < points1.size(); ++i)
            {
                if(status[i])
                {
                    cv::circle(pre_iamge, points1[i], 5, cv::Scalar(255, 255, 0));
                    cv::circle(pre_iamge, points2[i], 3, cv::Scalar(255, 0, 0), -1);
                    cv::line(pre_iamge, points1[i], points2[i], cv::Scalar(255, 0, 255));
                }

            }
            cv::imshow("matches", pre_iamge);
            cv::waitKey(500);
            cv::waitKey(-1);
        }
        else
        {
            first_image = false;
        }

        pre_gray = cur_gray.clone();
    }

    return 0;
}