// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_POSE_DESCRIPTOR_HPP
#define QRB_POSE_DESCRIPTOR_HPP

#include "QC/Infras/Memory/BufferDescriptor.hpp"
#include "QRB/Memory/Common/Types.hpp"

#include <cstdint>
#include <string>

namespace QRB
{
namespace Memory
{

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
