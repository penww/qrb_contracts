// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_ODOMETRY_DESCRIPTOR_HPP
#define QRB_ODOMETRY_DESCRIPTOR_HPP

#include "QC/Common/QCString.hpp"
#include "QC/Infras/Memory/BufferDescriptor.hpp"
#include "QRB/Memory/Common/Types.hpp"

#include <cstddef>
#include <cstdint>
#include <utility>

namespace QRB
{
namespace Memory
{

/**
 * @addtogroup RoboticsMemory Robotics Memory Descriptors
 * @{
 */

/**
 * @struct OdometryDescriptor_t
 * @brief Descriptor for a robotics odometry buffer.
 *
 * Extends @c QC::Memory::BufferDescriptor_t to carry a single odometry estimate (pose and twist with covariance) backed by a DMA buffer.
 * The layout mirrors ROS 2 @c nav_msgs/Odometry.
 *
 * Inherited Members from QC::Memory::BufferDescriptor_t:
 * @param name          Human-readable buffer name.
 * @param pBuf          Virtual address of the DMA buffer.
 * @param size          Total allocated size of the DMA buffer (bytes).
 * @param type          Buffer type tag (@c QC_BUFFER_TYPE_CUSTOM_4).
 * @param alignment     Alignment of the buffer.
 * @param cache         Cache attributes.
 * @param allocatorType Allocator used to allocate the buffer.
 * @param dmaHandle     DMA handle of the buffer.
 * @param pid           Process ID of the allocating process.
 * @param validSize     Number of bytes of valid data currently stored.
 * @param offset        Byte offset of the valid data within the DMA buffer.
 * @param id            User-assigned identifier to distinguish the buffer.
 *
 * New Members:
 * @param timestampNs        Acquisition timestamp in nanoseconds (CLOCK_MONOTONIC),
 *                           matching ROS 2 std_msgs/Header.stamp.
 * @param frameId            Coordinate frame in which the pose is expressed
 *                           (e.g. "odom"), matching ROS 2 std_msgs/Header.frame_id. A
 *                           @c FrameId_t, a fixed-capacity string of at most
 *                           QRB_FRAME_ID_MAX_LEN characters.
 * @param childFrameId       Coordinate frame in which the twist is expressed
 *                           (typically the robot body frame, e.g. "base_link"). A
 *                           @c FrameId_t.
 * @param poseWithCovariance Estimated 6-DOF pose of @c childFrameId in @c frameId.
 * @param twistWithCovariance Estimated velocity, expressed in @c childFrameId.
 *
 * @note The descriptor has no dynamic length member (no std::string / std::vector), so copying it
 * never allocates, as required for buffer descriptors. The odometry values are members of the
 * descriptor, not stored in the DMA buffer.
 */
typedef struct OdometryDescriptor : public QC::Memory::BufferDescriptor_t
{
public:
    OdometryDescriptor() : QC::Memory::BufferDescriptor_t(), timestampNs( 0 )
    {
        type = QC::QC_BUFFER_TYPE_CUSTOM_4;
    }

    ~OdometryDescriptor() override = default;

    OdometryDescriptor( const OdometryDescriptor &other ) = default;
    OdometryDescriptor( OdometryDescriptor &&other ) = default;

    // Inherit the other assignment overloads of the base class, as qcnode descriptors do.
    using QC::Memory::BufferDescriptor_t::operator=;

    /**
     * @brief Copy-assign from a base @c BufferDescriptor_t.
     *
     * Copies all base-class fields; odometry-specific fields are left
     * unchanged so the caller can set them afterwards.
     */
    OdometryDescriptor &operator=( const QC::Memory::BufferDescriptor_t &other ) noexcept override;

    /**
     * @brief Copy-assign from another @c OdometryDescriptor_t.
     */
    OdometryDescriptor &operator=( const OdometryDescriptor &other ) noexcept;

    /**
     * @brief Move-assign from another @c OdometryDescriptor_t.
     */
    OdometryDescriptor &operator=( OdometryDescriptor &&other ) noexcept;

    /**
     * @brief Copy-assign from the root @c QCBufferDescriptorBase_t.
     */
    OdometryDescriptor &operator=( const QC::Memory::QCBufferDescriptorBase_t &other ) noexcept override;

    // -------------------------------------------------------------------------
    // Data members
    // -------------------------------------------------------------------------

    uint64_t              timestampNs;          /**< Acquisition timestamp (ns, CLOCK_MONOTONIC). */
    FrameId_t             frameId;              /**< Frame of the pose (e.g. "odom"). */
    FrameId_t             childFrameId;         /**< Frame of the twist (e.g. "base_link"). */
    PoseWithCovariance_t  poseWithCovariance;   /**< Pose with covariance. */
    TwistWithCovariance_t twistWithCovariance;  /**< Twist with covariance. */

} OdometryDescriptor_t;

inline OdometryDescriptor &OdometryDescriptor::operator=( const QC::Memory::BufferDescriptor_t &other ) noexcept
{
    QC::Memory::BufferDescriptor_t::operator=( other );
    this->type = QC::QC_BUFFER_TYPE_CUSTOM_4;
    return *this;
}

inline OdometryDescriptor &OdometryDescriptor::operator=( const OdometryDescriptor &other ) noexcept
{
    if ( this != &other )
    {
        QC::Memory::BufferDescriptor_t::operator=( other );
        this->type = QC::QC_BUFFER_TYPE_CUSTOM_4;
        this->timestampNs = other.timestampNs;
        this->frameId = other.frameId;
        this->childFrameId = other.childFrameId;
        this->poseWithCovariance = other.poseWithCovariance;
        this->twistWithCovariance = other.twistWithCovariance;
    }
    return *this;
}

inline OdometryDescriptor &OdometryDescriptor::operator=( OdometryDescriptor &&other ) noexcept
{
    // No heap buffer to steal: moving is a copy of the fixed-size members.
    if ( this != &other )
    {
        QC::Memory::BufferDescriptor_t::operator=( std::move( other ) );
        this->type = QC::QC_BUFFER_TYPE_CUSTOM_4;
        this->timestampNs = other.timestampNs;
        this->frameId = other.frameId;
        this->childFrameId = other.childFrameId;
        this->poseWithCovariance = other.poseWithCovariance;
        this->twistWithCovariance = other.twistWithCovariance;
    }
    return *this;
}

inline OdometryDescriptor &OdometryDescriptor::operator=( const QC::Memory::QCBufferDescriptorBase_t &other ) noexcept
{
    if ( this != &other )
    {
        const OdometryDescriptor_t *pDesc = dynamic_cast<const OdometryDescriptor_t *>( &other );
        const QC::Memory::BufferDescriptor_t *pBufDesc =
                dynamic_cast<const QC::Memory::BufferDescriptor_t *>( &other );
        if ( nullptr != pDesc )
        {
            OdometryDescriptor::operator=( *pDesc );
        }
        else if ( nullptr != pBufDesc )
        {
            OdometryDescriptor::operator=( *pBufDesc );
        }
        else
        {
            QC::Memory::BufferDescriptor_t::operator=( other );
        }
    }
    return *this;
}

/** @} */  // end of RoboticsMemory

}   // namespace Memory
}   // namespace QRB

#endif   // QRB_ODOMETRY_DESCRIPTOR_HPP
