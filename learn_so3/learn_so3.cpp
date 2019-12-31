#include <iostream>
#include <cmath>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <unsupported/Eigen/MatrixFunctions>
#include "sophus/se3.hpp"

using namespace std;
using namespace Eigen;

int main(int argc, char **argv) {

  // 90 degrees rot around Z axis
  Matrix3d R = AngleAxisd(M_PI / 4, Vector3d(0, 0, 1)).toRotationMatrix();
  Quaterniond q(R);
  Sophus::SO3d SO3_R(R);

  Vector3d w(0.01, 0.02, 0.03);

  // auto SO3_R2 = Sophus::SO3d::exp(w);
  // cout << "R_update = " << (R * SO3_R2.matrix()) << endl;   // same as follow

  Matrix3d tmp = Sophus::SO3d::hat(w);
  auto R2 = tmp.exp();
  cout << "R_update:\n" << R * R2 << endl;

  Quaterniond q2(1, 0.005, 0.01, 0.015);
  q2.normalize();
  auto q_update = q * q2;
  q_update.normalize();

  cout << "q_update:\n" << Eigen::Matrix3d(q_update)  << endl;

  cout << "diff:\n" << (R * R2 - Eigen::Matrix3d(q_update)) << endl;
  return 0;
}

