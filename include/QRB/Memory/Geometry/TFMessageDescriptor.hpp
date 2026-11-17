// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_TF_MESSAGE_DESCRIPTOR_HPP
#define QRB_TF_MESSAGE_DESCRIPTOR_HPP

#include "QC/Infras/Memory/BufferDescriptor.hpp"
#include "QRB/Memory/Common/Types.hpp"

#include <cstddef>
#include <cstdint>
#include <type_traits>
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
 * @struct TransformStamped_t
 * @brief POD representation of a single stamped transform stored in the DMA buffer,
 *        matching ROS 2 geometry_msgs/TransformStamped.
 *
 * @param timestampNs   Acquisition timestamp in nanoseconds (CLOCK_MONOTONIC),
 *                      matching ROS 2 TransformStamped.header.stamp.
 * @param frameId       Parent (reference) frame, matching TransformStamped.header.frame_id.
 * @param childFrameId  Child frame whose origin is described in @c frameId,
 *                      matching TransformStamped.child_frame_id.
 * @param transform     Translation and rotation from @c frameId to @c childFrameId.
 */
typedef struct TransformStamped
{
    uint64_t    timestampNs{ 0 };
    FrameId_t   frameId;
    FrameId_t   childFrameId;
    Transform_t transform;
} TransformStamped_t;

static_assert( std::is_trivially_copyable<TransformStamped_t>::value,
               "TransformStamped_t must be trivially copyable for DMA buffer storage" );
static_assert( std::is_standard_layout<TransformStamped_t>::value,
               "TransformStamped_t must be standard layout for DMA buffer storage" );

/**
 * @struct TFMessageProps_t
 * @brief Allocation request for a robotics TF message buffer.
 *
 * Only what determines the memory size is kept here: the number of transforms the buffer must be
 * able to hold. The number of transforms actually stored is @c TFMessageDescriptor_t::transformCount,
 * set by the producer after the buffer has been allocated.
 *
 * Inherited Members from QC::Memory::BufferProps_t:
 * @param size          Total required buffer size in bytes, set to
 *                      @c transformCount * sizeof( TransformStamped_t ).
 * @param alignment     Alignment requirement of the buffer.
 * @param cache         Cache attributes of the buffer.
 * @param allocatorType Allocator used to allocate the buffer.
 *
 * New Members:
 * @param transformCount  Number of transforms the buffer must be able to hold.
 *
 * @note Call @c UpdateSize() after changing @c transformCount directly.
 */
typedef struct TFMessageProps : public QC::Memory::BufferProps_t
{
public:
    /**
     * @brief Default constructor. Creates an empty request with size 0.
     */
    TFMessageProps() : QC::Memory::BufferProps_t(), transformCount( 0 ) {}

    ~TFMessageProps() override = default;

    TFMessageProps( const TFMessageProps & ) = default;
    TFMessageProps &operator=( const TFMessageProps & ) = default;
    TFMessageProps( TFMessageProps && ) = default;
    TFMessageProps &operator=( TFMessageProps && ) = default;

    /**
     * @brief Constructor for TFMessageProps.
     * @param count  Number of transforms the buffer must be able to hold.
     * @param requestedAllocatorType  The allocator type, default is QC_MEMORY_ALLOCATOR_DMA.
     * @param requestedCache  The cache attributes, default is QC_CACHEABLE.
     * @note @c size is set to @c count * sizeof( TransformStamped_t ).
     */
    explicit TFMessageProps( uint32_t count,
                             QC::Memory::QCMemoryAllocator_e requestedAllocatorType = QC::Memory::QC_MEMORY_ALLOCATOR_DMA,
                             QC::Memory::QCAllocationCache_e requestedCache = QC::Memory::QC_CACHEABLE )
        : QC::Memory::BufferProps_t( 0, requestedAllocatorType, requestedCache ),
          transformCount( count )
    {
        UpdateSize();
    }

    /**
     * @brief Sets @c size to @c transformCount * sizeof( TransformStamped_t ).
     */
    void UpdateSize()
    {
        size = static_cast<size_t>( transformCount ) * sizeof( TransformStamped_t );
    }

    uint32_t transformCount;   /**< Number of transforms the buffer must be able to hold. */

} TFMessageProps_t;

/**
 * @struct TFMessageDescriptor_t
 * @brief Descriptor for a robotics TF message buffer, a batch of stamped transforms.
 *
 * Extends @c QC::Memory::BufferDescriptor_t to carry a list of stamped transforms packed into a
 * DMA buffer as a contiguous array of @c TransformStamped_t. The layout mirrors ROS 2
 * @c tf2_msgs/TFMessage. Nodes exchange the whole set of transforms of one update instead of a
 * single transform.
 *
 * Buffer layout:
 * @code
 *   pBuf → [ TransformStamped_t[0] | TransformStamped_t[1] | ... | TransformStamped_t[transformCount-1] ]
 * @endcode
 *
 * Inherited Members from QC::Memory::BufferDescriptor_t:
 * @param name          Human-readable buffer name.
 * @param pBuf          Virtual address of the DMA buffer.
 * @param size          Total allocated size of the DMA buffer (bytes).
 * @param type          Buffer type tag (@c QC_BUFFER_TYPE_CUSTOM_4, shared with the other
 *                      descriptors without a tag of their own, tell them apart by C++ type).
 * @param alignment     Alignment of the buffer.
 * @param cache         Cache attributes.
 * @param allocatorType Allocator used to allocate the buffer.
 * @param dmaHandle     DMA handle of the buffer.
 * @param pid           Process ID of the allocating process.
 * @param validSize     Number of bytes of valid transform data currently stored.
 * @param offset        Byte offset of the valid data within the DMA buffer.
 * @param id            User-assigned identifier to distinguish the buffer.
 *
 * New Members:
 * @param transformCount  Number of valid @c TransformStamped_t entries in the DMA buffer.
 *
 * @note There is no timestamp or frame id member: every transform carries its own timestamp,
 * parent frame and child frame, as in ROS 2.
 */
typedef struct TFMessageDescriptor : public QC::Memory::BufferDescriptor_t
{
public:
    TFMessageDescriptor() : QC::Memory::BufferDescriptor_t(), transformCount( 0 )
    {
        type = QC::QC_BUFFER_TYPE_CUSTOM_4;
    }

    ~TFMessageDescriptor() override = default;

    TFMessageDescriptor( const TFMessageDescriptor &other ) = default;
    TFMessageDescriptor( TFMessageDescriptor &&other ) = default;

    // Inherit the other assignment overloads of the base class, as qcnode descriptors do.
    using QC::Memory::BufferDescriptor_t::operator=;

    /**
     * @brief Copy-assign from a base @c BufferDescriptor_t.
     *
     * Copies all base-class fields; TF-specific fields are left unchanged so the caller can set
     * them afterwards.
     */
    TFMessageDescriptor &operator=( const QC::Memory::BufferDescriptor_t &other ) noexcept override;

    /**
     * @brief Copy-assign from another @c TFMessageDescriptor_t.
     */
    TFMessageDescriptor &operator=( const TFMessageDescriptor &other ) noexcept;

    /**
     * @brief Move-assign from another @c TFMessageDescriptor_t.
     */
    TFMessageDescriptor &operator=( TFMessageDescriptor &&other ) noexcept;

    /**
     * @brief Copy-assign from the root @c QCBufferDescriptorBase_t.
     */
    TFMessageDescriptor &operator=( const QC::Memory::QCBufferDescriptorBase_t &other ) noexcept override;

    // -------------------------------------------------------------------------
    // Accessors
    // -------------------------------------------------------------------------

    /**
     * @brief Returns a pointer to the @p index -th transform in the DMA buffer.
     *
     * @param index  Zero-based transform index.
     * @return Pointer to the requested @c TransformStamped_t, or @c nullptr if @c pBuf is
     *         @c nullptr or @p index is not smaller than @c transformCount.
     */
    TransformStamped_t *GetTransformPtr( uint32_t index ) const
    {
        TransformStamped_t *pTransform = nullptr;
        if ( ( pBuf != nullptr ) && ( index < transformCount ) )
        {
            pTransform = reinterpret_cast<TransformStamped_t *>( GetDataPtr() ) + index;
        }
        return pTransform;
    }

    /**
     * @brief Returns the minimum DMA buffer size required to hold @p count transforms.
     *
     * @param count  Number of transforms to accommodate.
     * @return Required buffer size in bytes.
     */
    static size_t CalcBufferSize( uint32_t count )
    {
        return static_cast<size_t>( count ) * sizeof( TransformStamped_t );
    }

    // -------------------------------------------------------------------------
    // Data members
    // -------------------------------------------------------------------------

    uint32_t transformCount;   /**< Number of valid transforms in the DMA buffer. */

} TFMessageDescriptor_t;

inline TFMessageDescriptor &TFMessageDescriptor::operator=( const QC::Memory::BufferDescriptor_t &other ) noexcept
{
    QC::Memory::BufferDescriptor_t::operator=( other );
    this->type = QC::QC_BUFFER_TYPE_CUSTOM_4;
    return *this;
}

inline TFMessageDescriptor &TFMessageDescriptor::operator=( const TFMessageDescriptor &other ) noexcept
{
    if ( this != &other )
    {
        QC::Memory::BufferDescriptor_t::operator=( other );
        this->type = QC::QC_BUFFER_TYPE_CUSTOM_4;
        this->transformCount = other.transformCount;
    }
    return *this;
}

inline TFMessageDescriptor &TFMessageDescriptor::operator=( TFMessageDescriptor &&other ) noexcept
{
    if ( this != &other )
    {
        QC::Memory::BufferDescriptor_t::operator=( std::move( other ) );
        this->type = QC::QC_BUFFER_TYPE_CUSTOM_4;
        this->transformCount = other.transformCount;
    }
    return *this;
}

inline TFMessageDescriptor &TFMessageDescriptor::operator=( const QC::Memory::QCBufferDescriptorBase_t &other ) noexcept
{
    if ( this != &other )
    {
        const TFMessageDescriptor_t *pDesc = dynamic_cast<const TFMessageDescriptor_t *>( &other );
        const QC::Memory::BufferDescriptor_t *pBufDesc =
                dynamic_cast<const QC::Memory::BufferDescriptor_t *>( &other );
        if ( nullptr != pDesc )
        {
            TFMessageDescriptor::operator=( *pDesc );
        }
        else if ( nullptr != pBufDesc )
        {
            TFMessageDescriptor::operator=( *pBufDesc );
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

#endif   // QRB_TF_MESSAGE_DESCRIPTOR_HPP
