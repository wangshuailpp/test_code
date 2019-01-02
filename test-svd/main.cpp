#include <iostream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <fstream>

using namespace std;

int main() {

    //制作一组数据
    Eigen::Matrix2d A = Eigen::Matrix2d::Zero();
    A(0,0) = 2;
    A(0,1) = 1;
    A(1,0) = 3;
    A(1,1) = 2;
    Eigen::Vector2d b = Eigen::Vector2d::Zero();
    b(0,0) = 0;
    b(1,0) = 2;



    Eigen::JacobiSVD<Eigen::Matrix2d> svd(A, Eigen::ComputeFullU | Eigen::ComputeFullV);
//    Eigen::Vector2d x = svd.solve(b);
    Eigen::Vector2d x = svd.matrixV().col(1);
    cout << x.transpose() << endl;


    return 0;
}