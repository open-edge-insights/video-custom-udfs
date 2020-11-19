## GVA Safety Gear Ingestion

This container is based out of VideoIngestion container. Since GVA elements are used for analytics there is no need for using any udf.

 * `Video File - Gstreamer ingestor with GVA elements`

      ```javascript
      {
        "type": "gstreamer",
        "pipeline": "multifilesrc loop=TRUE stop-index=0 location=./test_videos/Safety_Full_Hat_and_Vest.avi ! h264parse ! decodebin ! videoconvert ! video/x-raw,format=BGR ! gvadetect model=models/ref/frozen_inference_graph.xml ! appsink"
      }


 * `Generic Plugin - Gstreamer ingestor with GVA elements`

    ```javascript
     {
       "type": "gstreamer",
       "pipeline": "gencamsrc serial=<DEVICE_SERIAL_NUMBER> pixel-format=ycbcr422_8 width=1920 height=1080 exposure-time=3250 ! vaapipostproc format=bgrx ! gvadetect model=models/ref/frozen_inference_graph.xml ! videoconvert !  video/x-raw,format=BGR ! appsink"
     }
    ```

 * `RTSP camera - Gstreamer ingestor with GVA elements`

      ```javascript
      {
        "type": "gstreamer",
        "pipeline": "rtspsrc location=\"rtsp://admin:intel123@<RTSP CAMERA IP>:554/\" latency=100 ! rtph264depay ! h264parse ! vaapih264dec ! vaapipostproc format=bgrx ! gvadetect model=models/ref/frozen_inference_graph.xml ! videoconvert ! video/x-raw,format=BGR ! appsink"
      }
      ```

 * `USB camera - Gstreamer ingestor with GVA elements`

      ```javascript
      {
        "type": "gstreamer",
        "pipeline": "v4l2src ! decodebin ! videoconvert ! video/x-raw,format=BGR ! gvadetect model=models/ref/frozen_inference_graph.xml ! appsink"
      }
      ```

 * `RTSP simulated - Gstreamer ingestor with GVA elements`

      ```javascript
      {
        "type": "gstreamer",
        "pipeline": "rtspsrc location=\"rtsp://localhost:8554/\" latency=100 ! rtph264depay ! h264parse ! vaapih264dec ! vaapipostproc format=bgrx ! gvadetect model=models/ref/frozen_inference_graph.xml ! videoconvert ! video/x-raw,format=BGR ! appsink"
      }
      ```

**Refer [GVA-README](../../VideoIngestion/docs/gva_doc.md) for more information on GVA.**

**Refer [VideoIngestion-README](../../VideoIngestion/README.md) for more information on ingestor configurations.**
