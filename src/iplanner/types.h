#ifndef IPLANNER_TYPES_H_
#define IPLANNER_TYPES_H_

#include <Eigen/Dense>

namespace iplanner
{
// Vector types
using Vector2i = Eigen::Vector2i;
using Vector3i = Eigen::Vector3i;
using Vector4i = Eigen::Vector4i;

using Vector2f = Eigen::Vector2f;
using Vector3f = Eigen::Vector3f;
using Vector4f = Eigen::Vector4f;

using Vector2d = Eigen::Vector2d;
using Vector3d = Eigen::Vector3d;
using Vector4d = Eigen::Vector4d;

using VectorXf = Eigen::VectorXf;
using VectorXd = Eigen::VectorXd;

using Quaternionf = Eigen::Quaternionf;
using Quaterniond = Eigen::Quaterniond;

// Matrix types
using Matrix2f = Eigen::Matrix2f;
using Matrix2x3f = Eigen::Matrix<float, 2, 3>;
using Matrix2x4f = Eigen::Matrix<float, 2, 4>;

using Matrix3x2f = Eigen::Matrix<float, 3, 2>;
using Matrix3f = Eigen::Matrix3f;
using Matrix3x4f = Eigen::Matrix<float, 3, 4>;

using Matrix4x2f = Eigen::Matrix<float, 4, 2>;
using Matrix4x3f = Eigen::Matrix<float, 4, 3>;
using Matrix4f = Eigen::Matrix4f;

using Matrix3d = Eigen::Matrix3d;
using Matrix4d = Eigen::Matrix4d;

using MatrixXf = Eigen::MatrixXf;
using MatrixXd = Eigen::MatrixXd;

using Affine3f = Eigen::Affine3f;
using Affine3d = Eigen::Affine3d;

using AngleAxisf = Eigen::AngleAxisf;
using AngleAxisd = Eigen::AngleAxisd;

// Fixed sized eigen containers with STL vector
using Vector3dVector = std::vector<Eigen::Vector3d, Eigen::aligned_allocator<Eigen::Vector3d>>;

using Affine3fVector = std::vector<Eigen::Affine3f, Eigen::aligned_allocator<Eigen::Affine3f>>;
using Affine3dVector = std::vector<Eigen::Affine3d, Eigen::aligned_allocator<Eigen::Affine3d>>;
}

#endif // IPLANNER_TYPES_H_
