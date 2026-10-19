# qrb_contracts

Shared interface contracts for the QRB (Qualcomm Robotics) platform.
This repository defines the C++ buffer descriptors used to describe robotics
sensor data across nodes and services.

## Overview

`qrb_contracts` provides C++ buffer descriptor contracts:

| Layer | Location | Purpose |
|---|---|---|
| **C++ Descriptors** | `include/` | In-process / DMA buffer metadata, used between QCNodes |

The descriptors mirror [ROS 2](https://docs.ros.org/en/rolling/) message definitions
(`geometry_msgs`, `sensor_msgs`, `vision_msgs`) to allow zero-copy or low-overhead
mapping between the two representations.

## Directory Structure

```
qrb_contracts/
└── include/
    └── QRB/Memory/
        ├── Common/
        │   └── Types.hpp                      # Shared geometry & sensor value types
        ├── Geometry/
        │   └── PoseDescriptor.hpp             # PoseDescriptor
        ├── Sensor/
        │   └── PointCloudDescriptor.hpp       # PointCloudDescriptor
        └── Vision/
            └── ObjectDetectionDescriptor.hpp  # ObjectDetectionDescriptor
```

## C++ Descriptors

All descriptors inherit from `QC::Memory::BufferDescriptor_t` (defined in `qcnode`)
and live in the `QRB::Memory` namespace.

### Inheritance

```
QC::Memory::QCBufferDescriptorBase_t
└── QC::Memory::BufferDescriptor_t
    ├── QRB::Memory::PoseDescriptor_t          (Geometry/PoseDescriptor.hpp)
    ├── QRB::Memory::PointCloudDescriptor_t    (Sensor/PointCloudDescriptor.hpp)
    └── QRB::Memory::ObjectDetectionDescriptor_t  (Vision/ObjectDetectionDescriptor.hpp)
```

### Common Types (`Common/Types.hpp`)

Shared value types used across all descriptors, split into two groups:

**Geometry** — mirrors `geometry_msgs`:
`Point3d_t`, `Vector3d_t`, `Quaternion_t`, `Pose_t`, `Transform_t`,
`PoseWithCovariance_t`, `BoundingBox3d_t`

**Sensor** — mirrors `sensor_msgs`:
`PointFieldType_e`, `PointField_t`

### Descriptors

#### `PoseDescriptor_t`
Mirrors `geometry_msgs/PoseWithCovarianceStamped`. Carries a single stamped
6-DOF pose with a 6×6 covariance matrix.

#### `PointCloudDescriptor_t`
Mirrors `sensor_msgs/PointCloud2`. The raw point data is stored in the DMA
buffer (`pBuf`). Field layout is described by a `std::vector<PointField_t>`,
allowing arbitrary per-point field combinations (XYZ, XYZI, XYZRGB, etc.).

#### `ObjectDetectionDescriptor_t`
Mirrors `vision_msgs/Detection3DArray`. Detection results are stored as a
contiguous array of `Detection3dRaw_t` POD structs directly in the DMA buffer.
`objCount` records the number of valid entries.

Buffer layout:
```
pBuf → [ Detection3dRaw_t[0] | Detection3dRaw_t[1] | ... | Detection3dRaw_t[objCount-1] ]
```

The following limits are configurable via macros before including the header:

| Macro | Default | Description |
|---|---|---|
| `QRB_DETECTION_MAX_CLASS_ID_LEN` | `64` | Max class ID string length (bytes) |
| `QRB_DETECTION_MAX_ID_LEN` | `64` | Max tracking ID string length (bytes) |
| `QRB_DETECTION_MAX_RESULTS` | `8` | Max hypothesis results per detection |

## Dependencies

- **qcnode** — provides `QC::Memory::BufferDescriptor_t` and related base types

## License

Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.  
SPDX-License-Identifier: BSD-3-Clause-Clear
