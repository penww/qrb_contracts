// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_POINT_CLOUD_DESCRIPTOR_HPP
#define QRB_POINT_CLOUD_DESCRIPTOR_HPP

#include "QC/Infras/Memory/BufferDescriptor.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace QRB
{
namespace Memory
{

/**
 * @defgroup RoboticsMemory Robotics Memory Descriptors
 * @{
 */

/**
 * @enum PointFieldType_e
 * @brief Scalar data types for a point field, matching ROS 2 sensor_msgs/PointField constants.
 */
typedef enum
{
    POINT_FIELD_TYPE_INT8    = 1,
    POINT_FIELD_TYPE_UINT8   = 2,
    POINT_FIELD_TYPE_INT16   = 3,
    POINT_FIELD_TYPE_UINT16  = 4,
    POINT_FIELD_TYPE_INT32   = 5,
    POINT_FIELD_TYPE_UINT32  = 6,
    POINT_FIELD_TYPE_FLOAT32 = 7,
    POINT_FIELD_TYPE_FLOAT64 = 8,
    POINT_FIELD_TYPE_MAX     = UINT8_MAX
} PointFieldType_e;

/**
 * @struct PointField_t
 * @brief Describes one named field within a single point, matching ROS 2 sensor_msgs/PointField.
 *
 * @param name      Field name (e.g. "x", "y", "z", "intensity", "ring").
 * @param offset    Byte offset of this field from the start of a point.
 * @param datatype  Scalar type of the field values.
 * @param count     Number of elements of @c datatype in this field (usually 1).
 */
typedef struct PointField
{
    std::string    name;
    uint32_t       offset;
    PointFieldType_e datatype;
    uint32_t       count;

    PointField( std::string name, uint32_t offset, PointFieldType_e datatype, uint32_t count = 1 )
        : name( std::move( name ) ), offset( offset ), datatype( datatype ), count( count )
    {}
} PointField_t;


/**
 * @struct PointCloudDescriptor_t
 * @brief Descriptor for a robotics point cloud buffer.
 *
 * Extends @c QC::Memory::BufferDescriptor_t with the metadata needed to interpret
 * the raw byte buffer as an ordered or unordered point cloud.
 * The layout mirrors ROS 2 @c sensor_msgs/PointCloud2 for zero-copy mapping.
 *
 * Inherited Members from QC::Memory::BufferDescriptor_t:
 * @param name          Human-readable buffer name.
 * @param pBuf          Virtual address of the DMA buffer.
 * @param size          Total allocated size of the DMA buffer (bytes).
 * @param type          Buffer type tag (@c QC_BUFFER_TYPE_CUSTOM_0 for point cloud).
 * @param alignment     Alignment of the buffer.
 * @param cache         Cache attributes.
 * @param allocatorType Allocator used to allocate the buffer.
 * @param dmaHandle     DMA handle of the buffer.
 * @param pid           Process ID of the allocating process.
 * @param validSize     Number of bytes of valid point data currently stored.
 * @param offset        Byte offset of the valid data within the DMA buffer.
 * @param id            User-assigned identifier to distinguish the buffer.
 *
 * New Members:
 * @param height        Number of rows. 1 for unordered (flat) clouds.
 * @param width         Number of points per row. For unordered clouds equals total point count.
 * @param fields        Description of each field within a point.
 * @param isBigEndian   True if point data is big-endian.
 * @param pointStep     Byte stride between consecutive points (one row element).
 * @param rowStep       Byte stride between consecutive rows; equals @c pointStep * @c width.
 * @param isDense       True if the cloud contains no invalid (NaN/Inf) points.
 * @param timestampNs   Acquisition timestamp in nanoseconds (CLOCK_MONOTONIC).
 * @param frameId       Coordinate frame in which the points are expressed
 *                      (e.g. "lidar_link", "base_link").
 */
typedef struct PointCloudDescriptor : public QC::Memory::BufferDescriptor_t
{
public:
    PointCloudDescriptor()
        : QC::Memory::BufferDescriptor_t(),
          height( 1 ),
          width( 0 ),
          isBigEndian( false ),
          pointStep( 0 ),
          rowStep( 0 ),
          isDense( false ),
          timestampNs( 0 )
    {
        type = QC::QC_BUFFER_TYPE_CUSTOM_0;
    }

    ~PointCloudDescriptor() override = default;

    PointCloudDescriptor( const PointCloudDescriptor &other ) = default;

    /**
     * @brief Copy-assign from a base @c BufferDescriptor_t.
     *
     * Copies all base-class fields; point-cloud-specific fields are left
     * unchanged so the caller can set them afterwards.
     */
    PointCloudDescriptor &operator=( const QC::Memory::BufferDescriptor_t &other );

    /**
     * @brief Copy-assign from another @c PointCloudDescriptor_t.
     */
    PointCloudDescriptor &operator=( const PointCloudDescriptor &other );

    /**
     * @brief Copy-assign from the root @c QCBufferDescriptorBase_t.
     */
    PointCloudDescriptor &operator=( const QC::Memory::QCBufferDescriptorBase_t &other );

    // -------------------------------------------------------------------------
    // Accessors
    // -------------------------------------------------------------------------

    /**
     * @brief Returns the total number of points in the cloud (@c width * @c height).
     */
    uint32_t GetNumPoints() const { return width * height; }

    /**
     * @brief Returns a pointer to the first byte of the @p index -th point.
     *
     * No bounds checking is performed; the caller must ensure
     * @p index < GetNumPoints().
     *
     * @param index  Zero-based point index.
     * @return Pointer to the first byte of the requested point, or @c nullptr
     *         if @c pBuf is @c nullptr.
     */
    uint8_t *GetPointPtr( uint32_t index ) const
    {
        if ( pBuf == nullptr )
        {
            return nullptr;
        }
        return static_cast<uint8_t *>( GetDataPtr() ) + ( static_cast<size_t>( index ) * pointStep );
    }

    // -------------------------------------------------------------------------
    // Data members
    // -------------------------------------------------------------------------

    uint32_t                 height;       /**< Number of rows; 1 for unordered clouds. */
    uint32_t                 width;        /**< Points per row. */
    std::vector<PointField_t> fields;      /**< Per-field layout descriptors. */
    bool                     isBigEndian;  /**< True if point data is big-endian. */
    uint32_t                 pointStep;    /**< Bytes per point. */
    uint32_t                 rowStep;      /**< Bytes per row (pointStep * width). */
    bool                     isDense;      /**< True if no NaN/Inf points are present. */
    uint64_t                 timestampNs;  /**< Acquisition timestamp (ns, CLOCK_MONOTONIC). */
    std::string              frameId;      /**< Coordinate frame identifier. */

} PointCloudDescriptor_t;

/** @} */  // end of RoboticsMemory

}   // namespace Memory
}   // namespace QRB

#endif   // QRB_POINT_CLOUD_DESCRIPTOR_HPP
