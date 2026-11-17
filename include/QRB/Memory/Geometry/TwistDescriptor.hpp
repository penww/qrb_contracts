// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_TWIST_DESCRIPTOR_HPP
#define QRB_TWIST_DESCRIPTOR_HPP

#include "QC/Infras/Memory/BufferDescriptor.hpp"
#include "QRB/Memory/Common/Types.hpp"

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
 * @struct TwistDescriptor_t
 * @brief Descriptor for a robot velocity-command buffer.
 *
 * Extends @c QC::Memory::BufferDescriptor_t to carry a single stamped velocity command (linear and angular) backed by a DMA buffer.
 * The layout mirrors ROS 2 @c geometry_msgs/TwistStamped.
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
 * @param timestampNs  Command timestamp in nanoseconds (CLOCK_MONOTONIC),
 *                     matching ROS 2 std_msgs/Header.stamp.
 * @param frameId      Coordinate frame in which the twist is expressed
 *                     (e.g. "base_link"), matching ROS 2 std_msgs/Header.frame_id. A
 *                     @c FrameId_t, a fixed-capacity string of at most QRB_FRAME_ID_MAX_LEN
 *                     characters.
 * @param twist        Commanded linear (m/s) and angular (rad/s) velocity.
 *
 * @note The descriptor has no dynamic length member (no std::string / std::vector), so copying it
 * never allocates, as required for buffer descriptors. The twist values are members of the
 * descriptor, not stored in the DMA buffer.
 */
typedef struct TwistDescriptor : public QC::Memory::BufferDescriptor_t
{
public:
    TwistDescriptor() : QC::Memory::BufferDescriptor_t(), timestampNs( 0 )
    {
        type = QC::QC_BUFFER_TYPE_CUSTOM_4;
    }

    ~TwistDescriptor() override = default;

    TwistDescriptor( const TwistDescriptor &other ) = default;
    TwistDescriptor( TwistDescriptor &&other ) = default;

    // Inherit the other assignment overloads of the base class, as qcnode descriptors do.
    using QC::Memory::BufferDescriptor_t::operator=;

    /**
     * @brief Copy-assign from a base @c BufferDescriptor_t.
     *
     * Copies all base-class fields; twist-specific fields are left
     * unchanged so the caller can set them afterwards.
     */
    TwistDescriptor &operator=( const QC::Memory::BufferDescriptor_t &other ) noexcept override;

    /**
     * @brief Copy-assign from another @c TwistDescriptor_t.
     */
    TwistDescriptor &operator=( const TwistDescriptor &other ) noexcept;

    /**
     * @brief Move-assign from another @c TwistDescriptor_t.
     */
    TwistDescriptor &operator=( TwistDescriptor &&other ) noexcept;

    /**
     * @brief Copy-assign from the root @c QCBufferDescriptorBase_t.
     */
    TwistDescriptor &operator=( const QC::Memory::QCBufferDescriptorBase_t &other ) noexcept override;

    // -------------------------------------------------------------------------
    // Data members
    // -------------------------------------------------------------------------

    uint64_t    timestampNs;  /**< Command timestamp (ns, CLOCK_MONOTONIC). */
    FrameId_t   frameId;      /**< Frame in which the twist is expressed. */
    Twist_t     twist;        /**< Commanded velocity. */

} TwistDescriptor_t;

inline TwistDescriptor &TwistDescriptor::operator=( const QC::Memory::BufferDescriptor_t &other ) noexcept
{
    QC::Memory::BufferDescriptor_t::operator=( other );
    this->type = QC::QC_BUFFER_TYPE_CUSTOM_4;
    return *this;
}

inline TwistDescriptor &TwistDescriptor::operator=( const TwistDescriptor &other ) noexcept
{
    if ( this != &other )
    {
        QC::Memory::BufferDescriptor_t::operator=( other );
        this->type = QC::QC_BUFFER_TYPE_CUSTOM_4;
        this->timestampNs = other.timestampNs;
        this->frameId = other.frameId;
        this->twist = other.twist;
    }
    return *this;
}

inline TwistDescriptor &TwistDescriptor::operator=( TwistDescriptor &&other ) noexcept
{
    // No heap buffer to steal: moving is a copy of the fixed-size members.
    if ( this != &other )
    {
        QC::Memory::BufferDescriptor_t::operator=( std::move( other ) );
        this->type = QC::QC_BUFFER_TYPE_CUSTOM_4;
        this->timestampNs = other.timestampNs;
        this->frameId = other.frameId;
        this->twist = other.twist;
    }
    return *this;
}

inline TwistDescriptor &TwistDescriptor::operator=( const QC::Memory::QCBufferDescriptorBase_t &other ) noexcept
{
    if ( this != &other )
    {
        const TwistDescriptor_t *pDesc = dynamic_cast<const TwistDescriptor_t *>( &other );
        const QC::Memory::BufferDescriptor_t *pBufDesc =
                dynamic_cast<const QC::Memory::BufferDescriptor_t *>( &other );
        if ( nullptr != pDesc )
        {
            TwistDescriptor::operator=( *pDesc );
        }
        else if ( nullptr != pBufDesc )
        {
            TwistDescriptor::operator=( *pBufDesc );
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

#endif   // QRB_TWIST_DESCRIPTOR_HPP
