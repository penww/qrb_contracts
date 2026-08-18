// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_OBJECT_DETECTION_DESCRIPTOR_HPP
#define QRB_OBJECT_DETECTION_DESCRIPTOR_HPP

#include "QC/Infras/Memory/BufferDescriptor.hpp"
#include "QRB/Memory/Common/Types.hpp"

#include <cstdint>
#include <cstring>

namespace QRB
{
namespace Memory
{

/**
 * @addtogroup RoboticsMemory Robotics Memory Descriptors
 * @{
 */

/** @brief Maximum length of a class ID string (including null terminator). */
#ifndef QRB_DETECTION_MAX_CLASS_ID_LEN
#define QRB_DETECTION_MAX_CLASS_ID_LEN 64
#endif

/** @brief Maximum length of a detection tracking ID string (including null terminator). */
#ifndef QRB_DETECTION_MAX_ID_LEN
#define QRB_DETECTION_MAX_ID_LEN 64
#endif

/** @brief Maximum number of hypothesis results per detection. */
#ifndef QRB_DETECTION_MAX_RESULTS
#define QRB_DETECTION_MAX_RESULTS 8
#endif

/**
 * @struct ObjectHypothesisRaw_t
 * @brief POD representation of a single class hypothesis stored in the DMA buffer,
 *        matching ROS 2 vision_msgs/ObjectHypothesisWithPose.
 *
 * @param classId     Null-terminated class identifier string.
 * @param score       Confidence score in [0, 1].
 * @param pose        6-DOF pose with covariance of the hypothesised object.
 */
typedef struct ObjectHypothesisRaw
{
    char                 classId[QRB_DETECTION_MAX_CLASS_ID_LEN];
    double               score;
    PoseWithCovariance_t pose;
} ObjectHypothesisRaw_t;

static_assert( std::is_trivially_copyable<ObjectHypothesisRaw_t>::value,
               "ObjectHypothesisRaw_t must be trivially copyable for DMA buffer storage" );

/**
 * @struct Detection3dRaw_t
 * @brief POD representation of a single 3-D detection stored in the DMA buffer,
 *        matching ROS 2 vision_msgs/Detection3D.
 *
 * @param id           Null-terminated tracking identifier. May be empty.
 * @param bbox         3-D bounding box surrounding the object.
 * @param resultCount  Number of valid entries in @c results.
 * @param results      Class hypotheses; only the first @c resultCount entries are valid.
 */
typedef struct Detection3dRaw
{
    char                   id[QRB_DETECTION_MAX_ID_LEN];
    BoundingBox3d_t        bbox;
    uint32_t               resultCount;
    ObjectHypothesisRaw_t  results[QRB_DETECTION_MAX_RESULTS];
} Detection3dRaw_t;

static_assert( std::is_trivially_copyable<Detection3dRaw_t>::value,
               "Detection3dRaw_t must be trivially copyable for DMA buffer storage" );

/**
 * @struct ObjectDetectionDescriptor_t
 * @brief Descriptor for a robotics 3-D object detection buffer.
 *
 * Extends @c QC::Memory::BufferDescriptor_t to carry a list of 3-D detections
 * packed into a DMA buffer as a contiguous array of @c Detection3dRaw_t.
 * The layout mirrors ROS 2 @c vision_msgs/Detection3DArray.
 *
 * Buffer layout:
 * @code
 *   pBuf → [ Detection3dRaw_t[0] | Detection3dRaw_t[1] | ... | Detection3dRaw_t[objCount-1] ]
 * @endcode
 *
 * Inherited Members from QC::Memory::BufferDescriptor_t:
 * @param name          Human-readable buffer name.
 * @param pBuf          Virtual address of the DMA buffer.
 * @param size          Total allocated size of the DMA buffer (bytes).
 * @param type          Buffer type tag (@c QC_BUFFER_TYPE_CUSTOM_2 for object detection).
 * @param alignment     Alignment of the buffer.
 * @param cache         Cache attributes.
 * @param allocatorType Allocator used to allocate the buffer.
 * @param dmaHandle     DMA handle of the buffer.
 * @param pid           Process ID of the allocating process.
 * @param validSize     Number of bytes of valid detection data currently stored.
 * @param offset        Byte offset of the valid data within the DMA buffer.
 * @param id            User-assigned identifier to distinguish the buffer.
 *
 * New Members:
 * @param timestampNs  Acquisition timestamp in nanoseconds (CLOCK_MONOTONIC),
 *                     matching ROS 2 std_msgs/Header.stamp.
 * @param frameId      Coordinate frame in which all detections are expressed
 *                     (e.g. "camera_link", "lidar_link"),
 *                     matching ROS 2 std_msgs/Header.frame_id.
 * @param objCount     Number of valid @c Detection3dRaw_t entries in the DMA buffer.
 */
typedef struct ObjectDetectionDescriptor : public QC::Memory::BufferDescriptor_t
{
public:
    ObjectDetectionDescriptor() : QC::Memory::BufferDescriptor_t(), timestampNs( 0 ), objCount( 0 )
    {
        type = QC::QC_BUFFER_TYPE_CUSTOM_2;
    }

    ~ObjectDetectionDescriptor() override = default;

    ObjectDetectionDescriptor( const ObjectDetectionDescriptor &other ) = default;

    /**
     * @brief Copy-assign from a base @c BufferDescriptor_t.
     *
     * Copies all base-class fields; detection-specific fields are left
     * unchanged so the caller can set them afterwards.
     */
    ObjectDetectionDescriptor &operator=( const QC::Memory::BufferDescriptor_t &other );

    /**
     * @brief Copy-assign from another @c ObjectDetectionDescriptor_t.
     */
    ObjectDetectionDescriptor &operator=( const ObjectDetectionDescriptor &other );

    /**
     * @brief Copy-assign from the root @c QCBufferDescriptorBase_t.
     */
    ObjectDetectionDescriptor &operator=( const QC::Memory::QCBufferDescriptorBase_t &other );

    // -------------------------------------------------------------------------
    // Accessors
    // -------------------------------------------------------------------------

    /**
     * @brief Returns a pointer to the @p index -th detection in the DMA buffer.
     *
     * No bounds checking is performed; the caller must ensure
     * @p index < @c objCount.
     *
     * @param index  Zero-based detection index.
     * @return Pointer to the requested @c Detection3dRaw_t, or @c nullptr if
     *         @c pBuf is @c nullptr.
     */
    Detection3dRaw_t *GetDetectionPtr( uint32_t index ) const
    {
        if ( pBuf == nullptr )
        {
            return nullptr;
        }
        return reinterpret_cast<Detection3dRaw_t *>( GetDataPtr() ) + index;
    }

    /**
     * @brief Returns the minimum DMA buffer size required to hold @p count detections.
     *
     * @param count  Number of detections to accommodate.
     * @return Required buffer size in bytes.
     */
    static size_t CalcBufferSize( uint32_t count )
    {
        return static_cast<size_t>( count ) * sizeof( Detection3dRaw_t );
    }

    // -------------------------------------------------------------------------
    // Data members
    // -------------------------------------------------------------------------

    uint64_t    timestampNs; /**< Acquisition timestamp (ns, CLOCK_MONOTONIC). */
    std::string frameId;     /**< Coordinate frame identifier. */
    uint32_t    objCount;    /**< Number of valid detections in the DMA buffer. */

} ObjectDetectionDescriptor_t;

/** @} */  // end of RoboticsMemory

}   // namespace Memory
}   // namespace QRB

#endif   // QRB_OBJECT_DETECTION_DESCRIPTOR_HPP
