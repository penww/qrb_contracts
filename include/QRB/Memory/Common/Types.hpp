// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_COMMON_TYPES_HPP
#define QRB_COMMON_TYPES_HPP

#include "QC/Common/QCString.hpp"

#include <array>
#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace QRB
{
namespace Memory
{

/**
 * @brief Maximum number of characters of a coordinate frame identifier (excluding the
 *        terminating null), shared by all descriptors.
 *
 * Fixed so that a buffer descriptor has no dynamic length member, as required by QCNode.
 */
static const std::size_t QRB_FRAME_ID_MAX_LEN = 63U;

/**
 * @brief Fixed-capacity coordinate frame identifier (e.g. "base_link", "odom", "lidar_link").
 *
 * A string longer than QRB_FRAME_ID_MAX_LEN is not truncated: QCString::Assign fails with
 * QC_STATUS_NOMEM and the string stays empty.
 */
typedef QC::QCString<QRB_FRAME_ID_MAX_LEN> FrameId_t;

/**
 * @defgroup RoboticsGeometry Robotics Geometry Types
 * @brief Fundamental geometric types shared across robotics buffer descriptors.
 *        Definitions mirror ROS 2 geometry_msgs to allow zero-copy mapping.
 * @{
 */

/**
 * @struct Point3d_t
 * @brief 3-D position vector, matching ROS 2 geometry_msgs/Point.
 *
 * @param x  X coordinate in metres.
 * @param y  Y coordinate in metres.
 * @param z  Z coordinate in metres.
 */
typedef struct Point3d
{
    double x{ 0.0 };
    double y{ 0.0 };
    double z{ 0.0 };
} Point3d_t;

/**
 * @struct Vector3d_t
 * @brief 3-D vector (e.g. velocity, acceleration, angular rate),
 *        matching ROS 2 geometry_msgs/Vector3.
 *
 * @param x  X component.
 * @param y  Y component.
 * @param z  Z component.
 */
typedef struct Vector3d
{
    double x{ 0.0 };
    double y{ 0.0 };
    double z{ 0.0 };
} Vector3d_t;

/**
 * @struct Quaternion_t
 * @brief Unit quaternion representing orientation,
 *        matching ROS 2 geometry_msgs/Quaternion.
 *
 * @param x  X component.
 * @param y  Y component.
 * @param z  Z component.
 * @param w  W (scalar) component. Default 1.0 represents identity rotation.
 */
typedef struct Quaternion
{
    double x{ 0.0 };
    double y{ 0.0 };
    double z{ 0.0 };
    double w{ 1.0 };
} Quaternion_t;

/**
 * @struct Pose_t
 * @brief 6-DOF pose (position + orientation),
 *        matching ROS 2 geometry_msgs/Pose.
 *
 * @param position     3-D position in the reference frame.
 * @param orientation  Orientation as a unit quaternion.
 */
typedef struct Pose
{
    Point3d_t    position;
    Quaternion_t orientation;
} Pose_t;

/**
 * @struct Transform_t
 * @brief Rigid-body transform (translation + rotation),
 *        matching ROS 2 geometry_msgs/Transform.
 *
 * @param translation  Translation vector.
 * @param rotation     Rotation as a unit quaternion.
 */
typedef struct Transform
{
    Vector3d_t   translation;
    Quaternion_t rotation;
} Transform_t;

/**
 * @struct PoseWithCovariance_t
 * @brief Pose with associated 6x6 covariance matrix,
 *        matching ROS 2 geometry_msgs/PoseWithCovariance.
 *
 * @param pose        6-DOF pose.
 * @param covariance  Row-major 6x6 covariance matrix for (x, y, z, rot_x, rot_y, rot_z).
 *                    A zero matrix indicates unknown covariance.
 */
typedef struct PoseWithCovariance
{
    Pose_t                 pose;
    std::array<double, 36> covariance{};
} PoseWithCovariance_t;

/**
 * @struct Twist_t
 * @brief Velocity expressed as linear and angular parts,
 *        matching ROS 2 geometry_msgs/Twist.
 *
 * @param linear   Linear velocity in m/s.
 * @param angular  Angular velocity in rad/s.
 */
typedef struct Twist
{
    Vector3d_t linear;
    Vector3d_t angular;
} Twist_t;

/**
 * @struct TwistWithCovariance_t
 * @brief Twist with associated 6x6 covariance matrix,
 *        matching ROS 2 geometry_msgs/TwistWithCovariance.
 *
 * @param twist       Velocity (linear + angular).
 * @param covariance  Row-major 6x6 covariance matrix for (x, y, z, rot_x, rot_y, rot_z).
 *                    A zero matrix indicates unknown covariance.
 */
typedef struct TwistWithCovariance
{
    Twist_t                twist;
    std::array<double, 36> covariance{};
} TwistWithCovariance_t;

/**
 * @struct BoundingBox3d_t
 * @brief A 3-D bounding box positioned and rotated about its center (6 DOF),
 *        matching ROS 2 vision_msgs/BoundingBox3D.
 *
 * @param center  Position and orientation of the bounding box center.
 * @param size    Total size of the box (metres) along each axis.
 */
typedef struct BoundingBox3d
{
    Pose_t     center;
    Vector3d_t size;
} BoundingBox3d_t;

/** @} */  // end of RoboticsGeometry

}   // namespace Memory
}   // namespace QRB

#endif   // QRB_COMMON_TYPES_HPP
