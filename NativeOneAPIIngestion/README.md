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

**NOTE:**

This is an example to demonstrate usage of oneAPI library in EII which uses [oneAPI Base Toolkit image](https://www.intel.com/content/www/us/en/developer/tools/oneapi/base-toolkit-download.html?operatingsystem=linux&distributions=aptpackagemanager) which includes all the required features of oneAPI. Because of this, the docker build time is approximately 2 hours and docker image size goes upto nearly 20GB+.

If one wants to use smaller oneAPI toolkit for thier respective usecase, they can do so by
pulling different oneAPI toolkits available at https://www.intel.com/content/www/us/en/developer/tools/oneapi/toolkits.html#gs.p5b1kr instead of `oneAPI Base Toolkit`.
