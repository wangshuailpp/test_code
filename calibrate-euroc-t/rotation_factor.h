//
// Created by SENSETIME\wangshuai1 on 18-9-12.
// 手动计算雅克比公式的代价函数
//

#ifndef UNTITLED_ROTATION_FACTOR_H
#define UNTITLED_ROTATION_FACTOR_H

#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>
#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include "sophus/so3.h"
#include "sophus/se3.h"
using namespace std;


template <typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 3, 3> skewSymmetric(const Eigen::MatrixBase<Derived> &q)
{
    Eigen::Matrix<typename Derived::Scalar, 3, 3> ans;
    ans << typename Derived::Scalar(0), -q(2), q(1),
            q(2), typename Derived::Scalar(0), -q(0),
            -q(1), q(0), typename Derived::Scalar(0);
    return ans;
}

template <typename Derived>
static Eigen::Quaternion<typename Derived::Scalar> positify(const Eigen::QuaternionBase<Derived> &q)
{
    //printf("a: %f %f %f %f", q.w(), q.x(), q.y(), q.z());
    //Eigen::Quaternion<typename Derived::Scalar> p(-q.w(), -q.x(), -q.y(), -q.z());
    //printf("b: %f %f %f %f", p.w(), p.x(), p.y(), p.z());
    //return q.template w() >= (typename Derived::Scalar)(0.0) ? q : Eigen::Quaternion<typename Derived::Scalar>(-q.w(), -q.x(), -q.y(), -q.z());
    return q;
}

template <typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qleft(const Eigen::QuaternionBase<Derived> &q)
{
    Eigen::Quaternion<typename Derived::Scalar> qq = positify(q);
    Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
    ans(0, 0) = qq.w(), ans.template block<1, 3>(0, 1) = -qq.vec().transpose();
    ans.template block<3, 1>(1, 0) = qq.vec(), ans.template block<3, 3>(1, 1) = qq.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() + skewSymmetric(qq.vec());
    return ans;
}

template <typename Derived>
static Eigen::Matrix<typename Derived::Scalar, 4, 4> Qright(const Eigen::QuaternionBase<Derived> &p)
{
    Eigen::Quaternion<typename Derived::Scalar> pp = positify(p);
    Eigen::Matrix<typename Derived::Scalar, 4, 4> ans;
    ans(0, 0) = pp.w(), ans.template block<1, 3>(0, 1) = -pp.vec().transpose();
    ans.template block<3, 1>(1, 0) = pp.vec(), ans.template block<3, 3>(1, 1) = pp.w() * Eigen::Matrix<typename Derived::Scalar, 3, 3>::Identity() - skewSymmetric(pp.vec());
    return ans;
}


//简单的旋转数据计算
//4是参数qvc的维度,待优化的量,3是残差的维度
class RotationFactorEasy : public ceres::SizedCostFunction<3, 4> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual ~RotationFactorEasy(){}
    RotationFactorEasy(Eigen::Quaterniond& _qc, Eigen::Quaterniond& _qw) : qc(_qc), qw(_qw) {}
    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const
    {
        //1.首先写出残差公式
//        cout << "qw:\n" << qw.w() << " " << qw.x() << " " << qw.y() << " " << qw.z() << endl;
//        cout << "qc:\n" << qc.w() << " " << qc.x() << " " << qc.y() << " " << qc.z() << endl;
        Eigen::Quaterniond qvc(parameters[0][3], parameters[0][0], parameters[0][1], parameters[0][2]);//四元数参数wxyz
//        cout << "qvc:\n" << qvc.w() << " " << qvc.x() << " " << qvc.y() << " " << qvc.z() << endl;
        Eigen::Quaterniond q_err = qc.inverse() * qw * qvc;
        cout << "q_err:\n" << q_err.w() << " " << q_err.x() << " " << q_err.y() << " " << q_err.z() << endl;
        Eigen::Vector3d v_err = 2 * q_err.vec();
        cout << "v_err:\n" << v_err(0) << " " << v_err(1) << " " << v_err(2) << endl;
        residuals[0] = v_err(0);
        residuals[1] = v_err(1);
        residuals[2] = v_err(2);

        //2.写出雅克比公式
        if(jacobians)
        {
            if(jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double,3,4,Eigen::RowMajor>> jacobian_rotation(jacobians[0]);
                jacobian_rotation.setZero();

                //解析求导
                jacobian_rotation.block<3,3>(0,0) = Qleft(qc.inverse() * qw * qvc).bottomRightCorner<3,3>();

                cout << "jacobian_rotation:\n" << jacobian_rotation << endl;

            }
        }

    }
    Eigen::Quaterniond qc;
    Eigen::Quaterniond qw;

};


//复杂的旋转数据计算
//4是参数qvc的维度,待优化的量,3是残差的维度
class RotationFactor : public ceres::SizedCostFunction<3, 4> {
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW
    virtual ~RotationFactor(){}
    RotationFactor(Eigen::Quaterniond& _qc, Eigen::Quaterniond& _qw) : qc(_qc), qw(_qw) {}
    virtual bool Evaluate(double const* const* parameters,
                          double* residuals,
                          double** jacobians) const
    {
        //1.首先写出残差公式
//        cout << "qw:\n" << qw.w() << " " << qw.x() << " " << qw.y() << " " << qw.z() << endl;
//        cout << "qc:\n" << qc.w() << " " << qc.x() << " " << qc.y() << " " << qc.z() << endl;
        Eigen::Quaterniond qvc(parameters[0][3], parameters[0][0], parameters[0][1], parameters[0][2]);//四元数参数wxyz
//        cout << "qvc:\n" << qvc.w() << " " << qvc.x() << " " << qvc.y() << " " << qvc.z() << endl;
        Eigen::Quaterniond q_err = qvc.inverse() * qw * qvc * qc.inverse();
        cout << "q_err:\n" << q_err.w() << " " << q_err.x() << " " << q_err.y() << " " << q_err.z() << endl;
        Eigen::Vector3d v_err = 2 * q_err.vec();
        cout << "v_err:\n" << v_err(0) << " " << v_err(1) << " " << v_err(2) << endl;
        residuals[0] = v_err(0);
        residuals[1] = v_err(1);
        residuals[2] = v_err(2);

        //2.写出雅克比公式
        if(jacobians)
        {
            if(jacobians[0])
            {
                Eigen::Map<Eigen::Matrix<double,3,4,Eigen::RowMajor>> jacobian_rotation(jacobians[0]);
                jacobian_rotation.setZero();
//                jacobian_rotation.block<3,3>(0,0) = ((qvc.inverse() * qw * qvc * qc.inverse()).inverse() *
//                        (-Qright(qvc.inverse()) * Qright(qw) * qvc * qc.inverse() + Qright(qc.inverse()) * Qleft(qvc) * qvc.inverse() * qw)).bottomRightCorner<3,3>();
                //y = h(f(x)*g(x)),对x求导
//                Eigen::Matrix3d h = (qvc.inverse() * qw * qvc * qc.inverse()).inverse().toRotationMatrix();
//                Eigen::Matrix3d f_d = (-Qright(qvc.inverse()) * Qright(qw)).bottomRightCorner<3,3>();
//                Eigen::Matrix3d g_d = (Qright(qc.inverse()) * Qleft(qvc)).bottomRightCorner<3,3>();
//                Eigen::Matrix3d f = (qvc.inverse() * qw).toRotationMatrix();
//                Eigen::Matrix3d g = (qvc * qc.inverse()).toRotationMatrix();
//                jacobian_rotation.block<3,3>(0,0) = h * (f_d * g + f * g_d);
                //数值求导
                Eigen::Quaterniond q_distur(1,0.01,0.01,0.01);//首先给定一个小扰动
                q_distur = q_distur.normalized();              //归一化这个扰动四元数
                Eigen::Quaterniond q_err_distur = (qvc * q_distur).inverse() * qw * qvc * q_distur * qc.inverse();
                Eigen::Vector3d v_err_distur = 2 * q_err_distur.vec();
                Eigen::Vector3d delta_err = v_err_distur - v_err;
                Eigen::Vector3d phi(q_distur.x(), q_distur.y(), q_distur.z());
                jacobian_rotation(0,0) = delta_err(0)/phi(0);
                jacobian_rotation(1,0) = delta_err(1)/phi(0);
                jacobian_rotation(2,0) = delta_err(2)/phi(0);
                jacobian_rotation(0,1) = delta_err(0)/phi(1);
                jacobian_rotation(1,1) = delta_err(1)/phi(1);
                jacobian_rotation(2,1) = delta_err(2)/phi(1);
                jacobian_rotation(0,2) = delta_err(0)/phi(2);
                jacobian_rotation(1,2) = delta_err(1)/phi(2);
                jacobian_rotation(2,2) = delta_err(2)/phi(2);
                cout << "jacobian_rotation:\n" << jacobian_rotation << endl;

            }
        }

    }
    Eigen::Quaterniond qc;
    Eigen::Quaterniond qw;

};





#endif //UNTITLED_ROTATION_FACTOR_H
