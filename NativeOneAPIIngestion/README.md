## OneAPI Ingestion Demo UDF

This udf accepts the frames and shows how to use a DPC++ kernel together with
oneAPI Library to perform a simple frame content blur.

`UDF config`:

```json
{
"name": "dpcpp_blur_udf",
"type": "native",
"height" : 50,
"width" : 50
}
```

**NOTE**:
* In the above config, `height` and `width` are used as the intensity of blurring the each pixels in each row and column.

Refer [udfs-README](https://github.com/open-edge-insights/video-common/blob/master/udfs/README.md) for more information on configs of other in-built udfs like dummy, fps and resize udfs.