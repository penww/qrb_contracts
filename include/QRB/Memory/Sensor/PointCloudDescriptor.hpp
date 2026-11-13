// Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
// SPDX-License-Identifier: BSD-3-Clause-Clear

#ifndef QRB_POINT_CLOUD_DESCRIPTOR_HPP
#define QRB_POINT_CLOUD_DESCRIPTOR_HPP

#include "QC/Common/QCString.hpp"
#include "QC/Infras/Memory/BufferDescriptor.hpp"
#include "QRB/Memory/Common/Types.hpp"

#include <algorithm>
#include <cstddef>
#include <cstdint>
#include <iterator>
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
 * @brief Maximum number of fields in one point. Fixed so that the descriptor has no dynamic
 *        length member, as required for buffer descriptors.
 */
static const uint32_t QRB_POINT_CLOUD_MAX_FIELDS = 16U;

/**
 * @brief Maximum number of characters of a point field name (excluding the terminating null).
 */
static const std::size_t QRB_POINT_FIELD_NAME_MAX_LEN = 31U;

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
 * @param name      Field name (e.g. "x", "y", "z", "intensity", "ring"), a fixed-capacity string
 *                  of at most QRB_POINT_FIELD_NAME_MAX_LEN characters.
 * @param offset    Byte offset of this field from the start of a point.
 * @param datatype  Scalar type of the field values.
 * @param count     Number of elements of @c datatype in this field (usually 1).
 *
 * @note The struct has no dynamic length member, so copying it never allocates.
 */
typedef struct PointField
{
    /**
     * @brief Default constructor. Creates an empty field named "" at offset 0 with one UINT8
     *        element, so arrays of PointField_t can be declared.
     */
    PointField() : name(), offset( 0U ), datatype( POINT_FIELD_TYPE_UINT8 ), count( 1U ) {}

    /**
     * @brief Constructor for PointField.
     * @param fieldName  Field name; truncated names are not accepted: a name longer than
     *                   QRB_POINT_FIELD_NAME_MAX_LEN leaves the name empty.
     * @param fieldOffset  Byte offset of the field from the start of a point.
     * @param fieldDatatype  Scalar type of the field values.
     * @param fieldCount  Number of elements of @p fieldDatatype, default 1.
     */
    PointField( const char *fieldName, uint32_t fieldOffset, PointFieldType_e fieldDatatype,
                uint32_t fieldCount = 1U )
        : name(), offset( fieldOffset ), datatype( fieldDatatype ), count( fieldCount )
    {
        (void) name.Assign( fieldName );
    }

    QC::QCString<QRB_POINT_FIELD_NAME_MAX_LEN> name;
    uint32_t         offset;
    PointFieldType_e datatype;
    uint32_t         count;
} PointField_t;

/**
 * @brief Returns the size in bytes of one element of the given point field type.
 * @param datatype  The point field scalar type.
 * @return Element size in bytes, or 0 for an unknown type.
 */
inline uint32_t GetPointFieldTypeSize( PointFieldType_e datatype )
{
    uint32_t typeSize = 0U;
    switch ( datatype )
    {
        case POINT_FIELD_TYPE_INT8:
        case POINT_FIELD_TYPE_UINT8:
            typeSize = 1U;
            break;
        case POINT_FIELD_TYPE_INT16:
        case POINT_FIELD_TYPE_UINT16:
            typeSize = 2U;
            break;
        case POINT_FIELD_TYPE_INT32:
        case POINT_FIELD_TYPE_UINT32:
        case POINT_FIELD_TYPE_FLOAT32:
            typeSize = 4U;
            break;
        case POINT_FIELD_TYPE_FLOAT64:
            typeSize = 8U;
            break;
        default:
            break;
    }
    return typeSize;
}

/**
 * @brief Returns the minimal point step that holds all @p fields without padding,
 *        i.e. max( field.offset + sizeof( field.datatype ) * field.count ).
 *
 * This is only a lower bound of the real @c pointStep: alignment padding or undeclared bytes after
 * the last field cannot be derived from @p fields. Use it only when creating a new buffer whose
 * layout you control; when matching an existing layout (lidar packets, PCL structs, ROS messages)
 * take @c pointStep from the data source.
 *
 * @param fields     Per-field layout descriptors.
 * @param numFields  Number of valid entries in @p fields, clamped to
 *                   QRB_POINT_CLOUD_MAX_FIELDS.
 * @return The packed point step in bytes, 0 if there are no fields or only unknown types.
 */
inline uint32_t GetPackedPointStep( const PointField_t *fields, uint32_t numFields )
{
    uint32_t packed = 0U;
    const uint32_t num = ( nullptr == fields ) ? 0U : std::min( numFields, QRB_POINT_CLOUD_MAX_FIELDS );
    for ( uint32_t i = 0U; i < num; i++ )
    {
        const PointField_t &field = fields[i];
        const uint32_t end = field.offset + GetPointFieldTypeSize( field.datatype ) * field.count;
        if ( end > packed )
        {
            packed = end;
        }
    }
    return packed;
}

/**
 * @struct PointCloudProps_t
 * @brief Allocation request for a robotics point cloud buffer.
 *
 * Inherited Members from QC::Memory::BufferProps_t:
 * @param size          Total required buffer size in bytes, set to @c rowStep * @c height.
 * @param alignment     Alignment requirement of the buffer.
 * @param cache         Cache attributes of the buffer.
 * @param allocatorType Allocator used to allocate the buffer.
 *
 * New Members:
 * @param height        Number of rows. 1 for unordered (flat) clouds.
 * @param rowStep       Byte stride between consecutive rows. The caller must ensure it is at
 *                      least @c pointStep * @c width; a larger value means each row carries
 *                      trailing padding, as allowed by ROS 2 sensor_msgs/PointCloud2.
 *
 * @note Call @c UpdateSize() after changing @c height or @c rowStep directly.
 */
typedef struct PointCloudProps : public QC::Memory::BufferProps_t
{
public:
    /**
     * @brief Default constructor. Creates an empty request with size 0.
     */
    PointCloudProps() : QC::Memory::BufferProps_t(), height( 1 ), rowStep( 0 ) {}

    ~PointCloudProps() override = default;

    PointCloudProps( const PointCloudProps & ) = default;
    PointCloudProps &operator=( const PointCloudProps & ) = default;
    PointCloudProps( PointCloudProps && ) = default;
    PointCloudProps &operator=( PointCloudProps && ) = default;

    /**
     * @brief Constructor for PointCloudProps.
     * @param cloudHeight   Number of rows, 1 for an unordered cloud.
     * @param cloudRowStep  Bytes per row, at least @c pointStep * @c width.
     * @param requestedAllocatorType  The allocator type, default is QC_MEMORY_ALLOCATOR_DMA.
     * @param requestedCache  The cache attributes, default is QC_CACHEABLE.
     * @note @c size is set to @c cloudRowStep * @c cloudHeight.
     */
    PointCloudProps( uint32_t cloudHeight, uint32_t cloudRowStep,
                     QC::Memory::QCMemoryAllocator_e requestedAllocatorType = QC::Memory::QC_MEMORY_ALLOCATOR_DMA,
                     QC::Memory::QCAllocationCache_e requestedCache = QC::Memory::QC_CACHEABLE )
        : QC::Memory::BufferProps_t( 0, requestedAllocatorType, requestedCache ),
          height( cloudHeight ),
          rowStep( cloudRowStep )
    {
        UpdateSize();
    }

    /**
     * @brief Sets @c size to @c rowStep * @c height.
     */
    void UpdateSize() { size = static_cast<size_t>( rowStep ) * static_cast<size_t>( height ); }

    uint32_t height;   /**< Number of rows; 1 for unordered clouds. */
    uint32_t rowStep;  /**< Bytes per row (>= pointStep * width). */

} PointCloudProps_t;

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
 * @param fields        Description of each field within a point, a fixed-size array of
 *                      QRB_POINT_CLOUD_MAX_FIELDS entries.
 * @param numFields     Number of valid entries in @c fields.
 * @param isBigEndian   True if point data is big-endian.
 * @param pointStep     Byte stride between consecutive points (one row element).
 * @param rowStep       Byte stride between consecutive rows; at least @c pointStep * @c width,
 *                      larger when rows carry trailing padding (as in ROS 2 PointCloud2).
 * @param isDense       True if the cloud contains no invalid (NaN/Inf) points.
 * @param timestampNs   Acquisition timestamp in nanoseconds (CLOCK_MONOTONIC).
 * @param frameId       Coordinate frame in which the points are expressed
 *                      (e.g. "lidar_link", "base_link"), a @c FrameId_t, a fixed-capacity string
 *                      of at most QRB_FRAME_ID_MAX_LEN characters.
 *
 * @note The descriptor has no dynamic length member (no std::string / std::vector), so copying it
 * never allocates, as required for buffer descriptors.
 */
typedef struct PointCloudDescriptor : public QC::Memory::BufferDescriptor_t
{
public:
    PointCloudDescriptor()
        : QC::Memory::BufferDescriptor_t(),
          height( 1 ),
          width( 0 ),
          numFields( 0 ),
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
    PointCloudDescriptor( PointCloudDescriptor &&other ) = default;

    // Inherit the other assignment overloads of the base class, as qcnode descriptors do.
    using QC::Memory::BufferDescriptor_t::operator=;

    /**
     * @brief Copy-assign from a base @c BufferDescriptor_t.
     *
     * Copies all base-class fields; point-cloud-specific fields are left
     * unchanged so the caller can set them afterwards.
     */
    PointCloudDescriptor &operator=( const QC::Memory::BufferDescriptor_t &other ) noexcept override;

    /**
     * @brief Copy-assign from another @c PointCloudDescriptor_t.
     */
    PointCloudDescriptor &operator=( const PointCloudDescriptor &other ) noexcept;

    /**
     * @brief Move-assign from another @c PointCloudDescriptor_t.
     */
    PointCloudDescriptor &operator=( PointCloudDescriptor &&other ) noexcept;

    /**
     * @brief Copy-assign from the root @c QCBufferDescriptorBase_t.
     */
    PointCloudDescriptor &operator=( const QC::Memory::QCBufferDescriptorBase_t &other ) noexcept override;

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

    uint32_t                  height;       /**< Number of rows; 1 for unordered clouds. */
    uint32_t                  width;        /**< Points per row. */
    PointField_t              fields[QRB_POINT_CLOUD_MAX_FIELDS]; /**< Per-field layout descriptors. */
    uint32_t                  numFields;    /**< Number of valid entries in fields. */
    bool                      isBigEndian;  /**< True if point data is big-endian. */
    uint32_t                  pointStep;    /**< Bytes per point. */
    uint32_t                  rowStep;      /**< Bytes per row (>= pointStep * width). */
    bool                      isDense;      /**< True if no NaN/Inf points are present. */
    uint64_t                  timestampNs;  /**< Acquisition timestamp (ns, CLOCK_MONOTONIC). */
    FrameId_t                 frameId;      /**< Coordinate frame identifier. */

} PointCloudDescriptor_t;

inline PointCloudDescriptor &PointCloudDescriptor::operator=( const QC::Memory::BufferDescriptor_t &other ) noexcept
{
    QC::Memory::BufferDescriptor_t::operator=( other );
    this->type = QC::QC_BUFFER_TYPE_CUSTOM_0;
    return *this;
}

inline PointCloudDescriptor &PointCloudDescriptor::operator=( const PointCloudDescriptor &other ) noexcept
{
    if ( this != &other )
    {
        QC::Memory::BufferDescriptor_t::operator=( other );
        this->type = QC::QC_BUFFER_TYPE_CUSTOM_0;
        this->height = other.height;
        this->width = other.width;
        this->numFields = other.numFields;
        (void) std::copy( std::begin( other.fields ), std::end( other.fields ),
                          std::begin( this->fields ) );
        this->isBigEndian = other.isBigEndian;
        this->pointStep = other.pointStep;
        this->rowStep = other.rowStep;
        this->isDense = other.isDense;
        this->timestampNs = other.timestampNs;
        this->frameId = other.frameId;
    }
    return *this;
}

inline PointCloudDescriptor &PointCloudDescriptor::operator=( PointCloudDescriptor &&other ) noexcept
{
    // No heap buffer to steal: moving is a copy of the fixed-size members.
    if ( this != &other )
    {
        QC::Memory::BufferDescriptor_t::operator=( std::move( other ) );
        this->type = QC::QC_BUFFER_TYPE_CUSTOM_0;
        this->height = other.height;
        this->width = other.width;
        this->numFields = other.numFields;
        (void) std::copy( std::begin( other.fields ), std::end( other.fields ),
                          std::begin( this->fields ) );
        this->isBigEndian = other.isBigEndian;
        this->pointStep = other.pointStep;
        this->rowStep = other.rowStep;
        this->isDense = other.isDense;
        this->timestampNs = other.timestampNs;
        this->frameId = other.frameId;
    }
    return *this;
}

inline PointCloudDescriptor &PointCloudDescriptor::operator=( const QC::Memory::QCBufferDescriptorBase_t &other ) noexcept
{
    if ( this != &other )
    {
        const PointCloudDescriptor_t *pCloudDesc = dynamic_cast<const PointCloudDescriptor_t *>( &other );
        const QC::Memory::BufferDescriptor_t *pBufDesc =
                dynamic_cast<const QC::Memory::BufferDescriptor_t *>( &other );
        if ( nullptr != pCloudDesc )
        {
            PointCloudDescriptor::operator=( *pCloudDesc );
        }
        else if ( nullptr != pBufDesc )
        {
            PointCloudDescriptor::operator=( *pBufDesc );
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

#endif   // QRB_POINT_CLOUD_DESCRIPTOR_HPP
