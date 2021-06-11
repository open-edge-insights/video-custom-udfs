## PCL Demo UDF

This udf accepts the realsense color and depth frames, converts them to rs2 frame type and generates pcl point cloud.


`UDF config`:

```json
{
    "name": "rs2pcl",
    "type": "raw_native"
}
```
Refer [udfs-README](../../common/video/udfs/README.md) for more information on configs of other in-built udfs like dummy, fps and resize udfs.

