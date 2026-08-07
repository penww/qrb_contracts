// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_POSE_DESCRIPTOR_HPP
#define QRB_POSE_DESCRIPTOR_HPP

#include "QC/Infras/Memory/BufferDescriptor.hpp"

#include <array>
#include <cstdint>
#include <string>

namespace QRB
{
namespace Memory
{

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

/** @} */  // end of RoboticsGeometry

/**
 * @addtogroup RoboticsMemory Robotics Memory Descriptors
 * @{
 */

/**
 * @struct PoseDescriptor_t
 * @brief Descriptor for a robotics pose buffer.
 *
 * Extends @c QC::Memory::BufferDescriptor_t to carry a single stamped pose with
 * covariance backed by a DMA buffer.
 * The layout mirrors ROS 2 @c geometry_msgs/PoseWithCovarianceStamped.
 *
 * Inherited Members from QC::Memory::BufferDescriptor_t:
 * @param name          Human-readable buffer name.
 * @param pBuf          Virtual address of the DMA buffer.
 * @param size          Total allocated size of the DMA buffer (bytes).
 * @param type          Buffer type tag (@c QC_BUFFER_TYPE_CUSTOM_1 for pose).
 * @param alignment     Alignment of the buffer.
 * @param cache         Cache attributes.
 * @param allocatorType Allocator used to allocate the buffer.
 * @param dmaHandle     DMA handle of the buffer.
 * @param pid           Process ID of the allocating process.
 * @param validSize     Number of bytes of valid pose data currently stored.
 * @param offset        Byte offset of the valid data within the DMA buffer.
 * @param id            User-assigned identifier to distinguish the buffer.
 *
 * New Members:
 * @param timestampNs        Acquisition timestamp in nanoseconds (CLOCK_MONOTONIC),
 *                           matching ROS 2 std_msgs/Header.stamp.
 * @param frameId            Coordinate frame in which the pose is expressed
 *                           (e.g. "map", "odom"), matching ROS 2 std_msgs/Header.frame_id.
 * @param poseWithCovariance 6-DOF pose with a 6x6 covariance matrix.
 */
typedef struct PoseDescriptor : public QC::Memory::BufferDescriptor_t
{
public:
    PoseDescriptor() : QC::Memory::BufferDescriptor_t(), timestampNs( 0 )
    {
        type = QC::QC_BUFFER_TYPE_CUSTOM_1;
    }

    ~PoseDescriptor() override = default;

    PoseDescriptor( const PoseDescriptor &other ) = default;

    /**
     * @brief Copy-assign from a base @c BufferDescriptor_t.
     *
     * Copies all base-class fields; pose-specific fields are left
     * unchanged so the caller can set them afterwards.
     */
    PoseDescriptor &operator=( const QC::Memory::BufferDescriptor_t &other );

    /**
     * @brief Copy-assign from another @c PoseDescriptor_t.
     */
    PoseDescriptor &operator=( const PoseDescriptor &other );

    /**
     * @brief Copy-assign from the root @c QCBufferDescriptorBase_t.
     */
    PoseDescriptor &operator=( const QC::Memory::QCBufferDescriptorBase_t &other );

    // -------------------------------------------------------------------------
    // Data members
    // -------------------------------------------------------------------------

    uint64_t             timestampNs;        /**< Acquisition timestamp (ns, CLOCK_MONOTONIC). */
    std::string          frameId;            /**< Coordinate frame identifier. */
    PoseWithCovariance_t poseWithCovariance; /**< 6-DOF pose with covariance. */

} PoseDescriptor_t;

/** @} */  // end of RoboticsMemory

}   // namespace Memory
}   // namespace QRB

#endif   // QRB_POSE_DESCRIPTOR_HPP
