
#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "rotation_factor.h"

using namespace std;

/*SVD方式求解*/
void CalibrationExRotation(vector<Eigen::Quaterniond> &q_ws, vector<Eigen::Quaterniond> &q_im)
{
        int size = q_ws.size() - 1;
        Eigen::MatrixXd A(size * 4,4);
        A.setZero();
        for(int i = 0; i < q_ws.size() - 1; i++)
        {
            //W世界坐标,左乘
            Eigen::Matrix4d temp_a_w;
            temp_a_w.setZero();

            Eigen::Quaterniond q_w1 = q_ws[i];
            Eigen::Quaterniond q_w2 = q_ws[i + 1];
            Eigen::Quaterniond delta_q_w = q_w1.inverse() * q_w2;
            cout << "delta_q_w:\n" << delta_q_w.vec().transpose() << " " << delta_q_w.w() << endl;

            double delta_w_w = delta_q_w.w();
            Eigen::Vector3d delta_xyz_w = delta_q_w.vec();
            //四元数以xyzw顺序存储的左乘公式
            temp_a_w.block<3,3>(0,0) = delta_w_w * Eigen::Matrix3d::Identity() + skewSymmetric(delta_xyz_w);
            temp_a_w.block<3,1>(0,3) = delta_xyz_w;
            temp_a_w.block<1,3>(3,0) = -delta_xyz_w.transpose();
            temp_a_w(3,3) = delta_w_w;
//            cout << "skewSymmetric(delta_xyz_w):\n" << skewSymmetric(delta_xyz_w) << endl;
//            cout << "delta_w_w:\n" << delta_w_w << endl;

            //四元数以wxyz顺序存储的左乘公式
//            temp_a_w.block<3,3>(1,1) = delta_w_w * Eigen::Matrix3d::Identity() - skewSymmetric(delta_xyz_w);
//            temp_a_w.block<1,3>(0,1) = -delta_xyz_w.transpose();
//            temp_a_w.block<3,1>(1,0) = delta_xyz_w;
//            temp_a_w(0,0) = delta_w_w;
            cout << "temp_a_w:\n" << temp_a_w << endl;

            //C相机坐标,右乘
            Eigen::Matrix4d temp_a_im;
            temp_a_im.setZero();

            Eigen::Quaterniond q_im1 = q_im[i];
            Eigen::Quaterniond q_im2 = q_im[i + 1];
            Eigen::Quaterniond delta_q_im = q_im1.inverse() * q_im2;
            cout << "delta_q_im:\n" << delta_q_im.vec().transpose() << " " << delta_q_im.w() << endl;

            double delta_w_im = delta_q_im.w();
            Eigen::Vector3d delta_xyz_im = delta_q_im.vec();
//            cout << "skewSymmetric(delta_xyz_im):\n" << skewSymmetric(delta_xyz_im) << endl;
//            cout << "delta_w_im:\n" << delta_w_im << endl;

            //四元数以xyzw顺序存储的右乘公式
            temp_a_im.block<3,3>(0,0) = delta_w_im * Eigen::Matrix3d::Identity() - skewSymmetric(delta_xyz_w);
            temp_a_im.block<3,1>(0,3) = delta_xyz_im;
            temp_a_im.block<1,3>(3,0) = -delta_xyz_im.transpose();
            temp_a_im(3,3) = delta_w_im;

            //四元数以wxyz顺序存储的右乘公式
//            temp_a_im.block<3,3>(1,1) = delta_w_im * Eigen::Matrix3d::Identity() + skewSymmetric(delta_xyz_im);
//            temp_a_im.block<1,3>(0,1) = -delta_xyz_im.transpose();
//            temp_a_im.block<3,1>(1,0) = delta_xyz_im;
//            temp_a_im(0,0) = delta_w_im;
            cout << "temp_a_im:\n" << temp_a_im << endl;

            double angular_distance = 180 / M_PI * delta_q_im.angularDistance(delta_q_w);//设置权值
            double weight = angular_distance > 5.0 ? 5.0 / angular_distance : 1.0;//权值暂时未使用

//            A.block<4,4>(i * 4,0) = weight * (temp_a_w - temp_a_im);
            A.block<4,4>(i * 4,0) = -temp_a_im + temp_a_w;


//            Eigen::Vector4d x(-0.0280348, 0.286249, 0.481564, 0.827872);//w,x,y,z
//            cout << "temp_a_w * x:\n" << (temp_a_w * x).transpose() << endl;
//            cout << "temp_a_im * x:\n" << (temp_a_im * x).transpose() << endl;


        }
        Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);

//        Eigen::Matrix4d u = svd.matrixU();
//        Eigen::Matrix4d v = svd.matrixV();
//        Eigen::Vector4d a = svd.singularValues();
//        Eigen::Matrix4d d = Eigen::Matrix4d::Zero();
//        d(0,0) = a(0);
//        d(1,1) = a(1);
//        d(2,2) = a(2);
//        d(3,3) = a(3);
//
//
//        cout << "u:\n" << u << endl;
//        cout << "v:\n" << v << endl;
//        cout << "a:\n" << a << endl;
//        cout << "d:\n" << d << endl;
//
//        cout << "udvt:\n" << u*d*v.transpose() << endl;
//        cout << "A:\n" << A << endl;


        Eigen::Matrix<double, 4, 1> x = svd.matrixV().col(3);
        Eigen::Quaterniond q = Eigen::Quaterniond(x);
        cout << "x:\n" << x.transpose() << endl;
        cout << "q:\n" << q.normalized().vec().transpose() << " " << q.normalized().w() << endl;

        Eigen::Vector3d rvi_cov;
        rvi_cov = svd.singularValues().tail<3>();
        cout << "ric_cov(1):" << rvi_cov(1) << endl;
        cout << "q: \n" << q.vec().transpose() << " " << q.w() << endl;
        cout << "q.inverse: \n" << q.inverse().vec().transpose() << " " << q.inverse().w() << endl;
        if (rvi_cov(1) > 0.25 )
        {
            cout << "calibr q_vc ok!" << endl;
        }

//        cout << A*x << endl;
}


/*自动求导的方法的代价函数定义*/
//定义旋转外参的代价函数的模型(使用仿函数)
struct ROTATION_COST
{
    ROTATION_COST(const Eigen::Quaterniond& qv,const Eigen::Quaterniond& qc) :
            qv(qv), qc(qc) {}
    //计算残差
    template <typename  T>
    bool operator()(const T* const qvc,//模型参数,四维
                    T* residual) const//残差是一个1*3的向量
    {
        //定义残差公式
        Eigen::Quaternion<T> q_v(T(qv.w()), T(qv.x()), T(qv.y()), T(qv.z()));
//        cout << "qv:\n" << q_v.w() << " "  << q_v.x() << " " << q_v.y() << " " << q_v.z() << endl;
        Eigen::Quaternion<T> qcinv(T(qc.w()), T(-qc.x()), T(-qc.y()), T(-qc.z()));
        Eigen::Quaternion<T> q_c(T(qc.w()), T(qc.x()), T(qc.y()), T(qc.z()));
//        cout << "qv:\n" << qcinv.w() << " "  << qcinv.x() << " " << qcinv.y() << " " << qcinv.z() << endl;
        Eigen::Quaternion<T> q_vc(qvc[0], qvc[1], qvc[2], qvc[3]);
        Eigen::Quaternion<T> q_cv(qvc[0], -qvc[1], -qvc[2], -qvc[3]);
        cout << "q_cv:\n" << q_cv.w() << " "  << q_cv.x() << " " << q_cv.y() << " " << q_cv.z() << endl;

        const Eigen::Quaternion<T> err = q_cv * q_v * q_vc * qcinv;
//        cout << "err:\n" << err.w() << " "  << err.x() << " " << err.y() << " " << err.y() << endl;

        residual[0] = err.w() - T(1);
        residual[1] = err.x();
        residual[2] = err.y();
        residual[3] = err.z();

//        cout << "residual:\n" << residual[0] << " " <<residual[1] << " " << residual[2] << endl;

        return true;
    }

    Eigen::Quaterniond qv;
    Eigen::Quaterniond qc;

};

/*自动求导的方法*/
void CalibrationExRotationWithCeres(vector<Eigen::Quaterniond> &q_ws, vector<Eigen::Quaterniond> &q_im)
{
    ceres::Problem problem;
    Eigen::AngleAxisd rotation_vector(M_PI/2, Eigen::Vector3d(0,0,1));
    Eigen::Quaterniond q_w_c = Eigen::Quaterniond(rotation_vector);
    Eigen::Quaterniond q_c_w = q_w_c.inverse();
    double qvc[4] = {1,0,0,0};
//    Eigen::Quaterniond qvc = Eigen::Quaterniond::Identity();
    for(int i = 0; i < q_ws.size(); i++)
    {
        cout << q_ws[i].w() << " " << q_ws[i].x() << " " << q_ws[i].y() << " " << q_ws[i].z() << endl;
        cout << (q_w_c * q_im[i] * q_c_w).w() << " " << (q_w_c * q_im[i] * q_c_w).x() << " " << (q_w_c * q_im[i] * q_c_w).y() << " " << (q_w_c * q_im[i] * q_c_w).z() << endl;

        ceres::CostFunction* cost_function = new ceres::AutoDiffCostFunction<ROTATION_COST, 4, 4>(
                new ROTATION_COST(q_ws[i],q_im[i])
        );
        problem.AddResidualBlock(
                cost_function,
                NULL,            // 核函数，这里不使用，为空
                qvc// 待估计参数
        );
    }
    // 配置求解器
    ceres::Solver::Options options;     // 这里有很多配置项可以填
    options.linear_solver_type = ceres::DENSE_NORMAL_CHOLESKY;  // 增量方程如何求解
    options.minimizer_progress_to_stdout = true;   // 输出到cout

    ceres::Solver::Summary summary;                // 优化信息

    ceres::Solve ( options, &problem, &summary );  // 开始优化
    cout << "qvc:\n" << qvc[0] << " " << qvc[1] << " " << qvc[2] << " " << qvc[3] << endl;
}


/*手动计算雅克比方法*/
void CalibrationExRotationWithCeresAnalytic(vector<Eigen::Quaterniond> &q_ws, vector<Eigen::Quaterniond> &q_im)
{
    ceres::Problem problem;
    double qvc[] = {0, 0, 0, 1};
    for(int i = 0; i < q_ws.size(); i++)
    {
        //简单的旋转数据计算
//        RotationFactorEasy* rotation_factor_easy = new RotationFactorEasy(q_im[i], q_ws[i]);
//        problem.AddResidualBlock(rotation_factor_easy, NULL, qvc);

        //复杂的旋转数据计算
        RotationFactor* rotation_factor = new RotationFactor(q_im[i], q_ws[i]);
        problem.AddResidualBlock(rotation_factor, NULL, qvc);
    }
    // 配置求解器
    ceres::Solver::Options options;     // 这里有很多配置项可以填
    options.linear_solver_type = ceres::DENSE_QR;  // 增量方程如何求解
    options.minimizer_progress_to_stdout = true;   // 输出到cout

    ceres::Solver::Summary summary;                // 优化信息

    ceres::Solve ( options, &problem, &summary );  // 开始优化
    cout << "qvc:\n" << qvc[0] << " " << qvc[1] << " " << qvc[2] << " " << qvc[3] << endl;
}



int main(int argc, char** argv) {

    google::InitGoogleLogging(argv[0]);

    vector<Eigen::Quaterniond> q_ws;
    vector<Eigen::Quaterniond> q_cs;
    Eigen::Quaterniond q_w = Eigen::Quaterniond::Identity();
    Eigen::Quaterniond q_c = Eigen::Quaterniond::Identity();
    Eigen::AngleAxisd rotation_vector(M_PI/4, Eigen::Vector3d(0,0,1));
    Eigen::Quaterniond q_w_c = Eigen::Quaterniond(rotation_vector);
    Eigen::Quaterniond q_c_w = q_w_c.inverse();
    Eigen::Quaterniond delta_q = Eigen::Quaterniond(1,0.1,0.2,0.3);
    delta_q = delta_q.normalized();
    cout << "delta_q:\n" << delta_q.vec().transpose() << " " << delta_q.w() << endl;
    cout <<"q_w_c:\n"<< q_w_c.w() << " " << q_w_c.x() << " " << q_w_c.y() << " " << q_w_c.z() << endl;

    //制作简单的旋转数据,参考坐标系相同
//    q_ws.push_back(q_w);
//    q_cs.push_back(q_w * q_w_c);
//    for(int i = 0; i < 2; i++)//数据量可手动调节
//    {
//        q_w = q_w * delta_q;
//        q_c = q_w * q_w_c;
//        q_ws.push_back(q_w);
//        q_cs.push_back(q_c);
//
//
//    }

//    //制作复杂的旋转数据,参考坐标系不同
    q_ws.push_back(q_w);
    q_cs.push_back(q_c_w * q_w * q_w_c);
    cout << "q_w_c: \n" << q_w_c.vec().transpose() << " " << q_w_c.w() << endl;
    cout << "q_c_w: \n" << q_c_w.vec().transpose() << " " << q_c_w.w() << endl;
    for(int i = 0; i < 2; i++)//数据量可手动调节
    {
        q_w = q_w * delta_q;
        q_c = q_c_w * q_w * q_w_c;
        q_ws.push_back(q_w);
        q_cs.push_back(q_c);
    }

    ofstream out_q_start("../result/start_w_c_r.txt", ios::app);
    for(int i = 0; i < q_ws.size(); i++)
    {
        out_q_start << i << " "
                    << q_ws[i].x() << " "
                    << q_ws[i].y() << " "
                    << q_ws[i].z() << " "
                    << q_ws[i].w() << " "
                    << q_cs[i].x() << " "
                    << q_cs[i].y() << " "
                    << q_cs[i].z() << " "
                    << q_cs[i].w() << endl;
    }
    out_q_start.close();

    CalibrationExRotation(q_ws, q_cs);//SVD方法
//    CalibrationExRotationWithCeres(q_ws, q_cs);//ceres自动求导方法
//    CalibrationExRotationWithCeresAnalytic(q_ws, q_cs);




    ofstream out_q_end("../result/end_w_c_r.txt", ios::app);
    for(int i = 0; i < q_ws.size(); i++)
    {
        Eigen::Quaterniond temp_q = q_w_c * q_cs[i] * q_w_c.inverse();
        out_q_end << i << " "
                    << q_ws[i].x() << " "
                    << q_ws[i].y() << " "
                    << q_ws[i].z() << " "
                    << q_ws[i].w() << " "
                    << temp_q.x() << " "
                    << temp_q.y() << " "
                    << temp_q.z() << " "
                    << temp_q.w() << endl;
    }
    out_q_end.close();

//    Eigen::Quaterniond q1(1,0,0,0);
//    Eigen::Quaterniond q2 = q_c_w.inverse() * q1 * q_c_w;;
//    Eigen::Quaterniond q11(1,0.1,0.2,0.3);
//    Eigen::Quaterniond q22 = q_c_w.inverse() * q11 * q_c_w;
//    Eigen::Quaterniond delta_q2 = q2.inverse() * q22;
////    Eigen::Quaterniond q22(1,0.2,-0.1,0.3);
//    cout << "delta_q2:\n" << delta_q2.vec() << " " << delta_q2.w() << endl;
//
//    //W,右乘
//    Eigen::Matrix4d temp_a_w;
//    temp_a_w.setZero();
//    double delta_w_w = delta_q.w();
//    Eigen::Vector3d delta_xyz_w = delta_q.vec();
//    temp_a_w.block<3,3>(1,1) = delta_w_w * Eigen::Matrix3d::Identity() - skewSymmetric(delta_xyz_w);
//    temp_a_w.block<3,1>(0,1) = -delta_xyz_w.transpose();
//    temp_a_w.block<1,3>(1,0) = delta_xyz_w;
//    temp_a_w(0,0) = delta_w_w;
//
//    //C,左乘
//    Eigen::Matrix4d temp_a_im;
//    temp_a_im.setZero();
//
//    double delta_w_im = delta_q2.w();
//    //            Eigen::Vector3d delta_xyz_im(delta_q_im.x(), delta_q_im.y(), delta_q_im.z());
//    Eigen::Vector3d delta_xyz_im = delta_q2.vec();
//    //            cout << "skewSymmetric(delta_xyz_im):\n" << skewSymmetric(delta_xyz_im) << endl;
//    //            cout << "delta_w_im:\n" << delta_w_im << endl;
//    temp_a_im.block<3,3>(1,1) = delta_w_im * Eigen::Matrix3d::Identity() + skewSymmetric(delta_xyz_im);
//    temp_a_im.block<3,1>(0,1) = -delta_xyz_im.transpose();
//    temp_a_im.block<1,3>(1,0) = delta_xyz_im;
//    temp_a_im(0,0) = delta_w_im;
//    Eigen::Matrix4d A = temp_a_im - temp_a_w;
//
//    Eigen::JacobiSVD<Eigen::MatrixXd> svd(A, Eigen::ComputeFullV || Eigen::ComputeFullU);
//    Eigen::Matrix<double, 4, 1> x = svd.matrixV().col(3);
//    Eigen::Quaterniond q = Eigen::Quaterniond(x);
//
//    Eigen::Vector3d rvi_cov;
//    rvi_cov = svd.singularValues().tail<3>();
//    cout << "ric_cov(1):" << rvi_cov(1) << endl;
//    cout << "q: \n" << q.vec().transpose() << " " << q.w() << endl;
//    cout << "q.inverse: \n" << q.inverse().vec().transpose() << " " << q.inverse().w() << endl;
//    if (rvi_cov(1) > 0.25 )
//    {
//        cout << "calibr q_vc ok!" << endl;
//    }


    return 0;
}