## Safety Gear Demo UDF

This udf accepts the frames, detects safety gear such as safety helmet, safety jacket in
the frame and any violations occuring.

> **NOTE**: This works well with only the
> [safe gear video file](../../VideoIngestion/test_videos/Safety_Full_Hat_and_Vest.avi).
> For camera usecase, proper tuning needs to be done to have the proper model
> built and used for inference.

`UDF config`:

```javascript
{
"name": "safety_gear_demo",
"type": "native",
"device": "CPU",
"model_xml": "common/udfs/native/safety_gear_demo/ref/frozen_inference_graph.xml",
"model_bin": "common/udfs/native/safety_gear_demo/ref/frozen_inference_graph.bin"
}
```

----
**NOTE**:
The above config works for both "CPU" and "GPU" devices after setting
appropriate `device` value. If the device in the above config is "HDDL" or
"MYRIAD", please use the below config where the model_xml and model_bin
files are different. Please set the "device" value appropriately based on
the device used for inferencing.

```javascript
{
"name": "safety_gear_demo",
"type": "native",
"device": "HDDL",
"model_xml": "common/udfs/native/safety_gear_demo/ref/frozen_inference_graph_fp16.xml",
"model_bin": "common/udfs/native/safety_gear_demo/ref/frozen_inference_graph_fp16.bin"
}
```

