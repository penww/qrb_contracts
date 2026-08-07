# qrb_contracts

Shared interface contracts for the QRB (Qualcomm Robotics) platform.
This repository defines the C++ buffer descriptors and Protocol Buffer schemas
used to describe robotics sensor data across nodes and services.

## Overview

`qrb_contracts` provides two layers of contracts:

| Layer | Location | Purpose |
|---|---|---|
| **C++ Descriptors** | `include/` | In-process / DMA buffer metadata, used between QCNodes |
| **Protobuf Schemas** | `proto/` | Serialization schemas for cross-machine transport (e.g. cloud upload) |

Both layers mirror [ROS 2](https://docs.ros.org/en/rolling/) message definitions
(`geometry_msgs`, `sensor_msgs`, `vision_msgs`) to allow zero-copy or low-overhead
mapping between the two representations.

## Directory Structure

```
qrb_contracts/
├── include/
│   └── QRB/Memory/
│       ├── Geometry/
│       │   └── PoseDescriptor.hpp          # Geometry types + PoseDescriptor
│       ├── Sensor/
│       │   └── PointCloudDescriptor.hpp    # PointCloudDescriptor
│       └── Vision/
│           └── ObjectDetectionDescriptor.hpp  # ObjectDetectionDescriptor
├── proto/
│   └── qrb/contract/
│       ├── geometry/v1/
│       │   ├── point3d.proto
│       │   ├── vector3d.proto
│       │   ├── quaternion.proto
│       │   ├── pose.proto
│       │   ├── pose_with_covariance.proto
│       │   └── pose_with_covariance_stamped.proto
│       ├── sensor/v1/
│       │   ├── point_field.proto
│       │   └── point_cloud2.proto
│       └── vision/v1/
│           ├── object_hypothesis.proto
│           ├── object_hypothesis_with_pose.proto
│           ├── detection_3d.proto
│           └── detection_3d_array.proto

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

### Descriptors

#### `PoseDescriptor_t`
Mirrors `geometry_msgs/PoseWithCovarianceStamped`. Carries a single stamped
6-DOF pose with a 6×6 covariance matrix.

Also defines the shared geometry value types used across descriptors:
`Point3d_t`, `Vector3d_t`, `Quaternion_t`, `Pose_t`, `Transform_t`,
`PoseWithCovariance_t`.

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

## Protobuf Schemas

Schemas are organised under `proto/qrb/contract/` with versioned sub-paths
(`v1/`) to allow non-breaking evolution. Each ROS 2 message type maps to one
`.proto` file.

Local-only fields (`pBuf`, `dmaHandle`, `pid`, `size`, `alignment`, `cache`,
`allocatorType`, `offset`, `validSize`) are intentionally omitted from all
proto messages.

### ROS 2 ↔ Proto mapping

| ROS 2 message | Proto file |
|---|---|
| `geometry_msgs/Point` | `geometry/v1/point3d.proto` |
| `geometry_msgs/Vector3` | `geometry/v1/vector3d.proto` |
| `geometry_msgs/Quaternion` | `geometry/v1/quaternion.proto` |
| `geometry_msgs/Pose` | `geometry/v1/pose.proto` |
| `geometry_msgs/PoseWithCovariance` | `geometry/v1/pose_with_covariance.proto` |
| `geometry_msgs/PoseWithCovarianceStamped` | `geometry/v1/pose_with_covariance_stamped.proto` |
| `sensor_msgs/PointField` | `sensor/v1/point_field.proto` |
| `sensor_msgs/PointCloud2` | `sensor/v1/point_cloud2.proto` |
| `vision_msgs/ObjectHypothesis` | `vision/v1/object_hypothesis.proto` |
| `vision_msgs/ObjectHypothesisWithPose` | `vision/v1/object_hypothesis_with_pose.proto` |
| `vision_msgs/Detection3D` | `vision/v1/detection_3d.proto` |
| `vision_msgs/Detection3DArray` | `vision/v1/detection_3d_array.proto` |

## Dependencies

- **qcnode** — provides `QC::Memory::BufferDescriptor_t` and related base types
- **Protocol Buffers** (`protoc` ≥ 3.15) — required to generate code from `.proto` files

## License

Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.  
SPDX-License-Identifier: BSD-3-Clause-Clear
