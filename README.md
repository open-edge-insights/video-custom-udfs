# **Introduction**
This document describes the new approach of creating UDFs and using them inside the EIS framework. Unlike [The UDF Writing Guide](../common/video/udfs/HOWTO_GUIDE_FOR_WRITING_UDF.md) which specifically emphasizes on the coding aspects(callbacks) of the UDFs, this document describes the workflow of a  custom UDF.

Currently the UDFs are to be created inside [udfs-path](../common/video/udfs) of EIS build environment so that it can get compiled into the VI(Video Ingestion) & VA(Video Analytics) containers. In addition to aforementioned approach,each UDF can be built as an independent container based out of VI(VideoIngestion) or VA(VideoAnalytics) container image. This additional method has multiple benefits, listing some of them below:

* With increased number of sample UDFs, VI and VA need not grow large in size because of the bloated Algo artifacts.
* Any update to the UDF's Algo or its logic will compile and build only intended UDF specific code, instead of rebuilding every UDFs
* Any change to base containers because of some unrelated changes in ubuntu packages triggers a complete container build again, this adds to build time of an UDF. 
* Every UDF can be versioned independently as they are represented by its own container.
* A reduced size of UDF container will reduce the network overhaul while transferring the images from REGISTRY to target system.

As per this approach an UDF or a chain of UDFs should be compiled and run as a separate EIS container. A video streaming pipeline contains two important components among all i.e. ingestion and analytics and in EIS user adds UDFs as pre-processing, post-processing or analytics Algo, hence these UDF containers need to ne inherited from VI and VA container.

# **UDF Container Directory Layout**
1. A native(c++) & python UDF container source base looks as below, though it can look different based on use-case. 

``` bash
NativeSafetyGearAnalytics
├── config.json
├── docker-compose.yml
├── Dockerfile
└── safety_gear_demo
    ├── CMakeLists.txt
    ├── ref
    │   ├── frozen_inference_graph.bin
    │   ├── frozen_inference_graph_fp16.bin
    │   ├── frozen_inference_graph_fp16.xml
    │   └── frozen_inference_graph.xml
    ├── safety_gear_demo.cpp
    └── safety_gear_demo.h
```
A typical python containers looks as below

``` bash
PyMultiClassificationIngestion
├── config.json
├── docker-compose.yml
├── Dockerfile
└── sample_classification
    ├── __init__.py
    ├── multi_class_classifier.py
    └── ref
        ├── squeezenet1.1_FP16.bin
        ├── squeezenet1.1_FP16.xml
        ├── squeezenet1.1_FP32.bin
        ├── squeezenet1.1_FP32.xml
        └── squeezenet1.1.labels
```
The top level directory ***"NativeSafetyGearAnalytics"*** & ***"PyMultiClassificationIngestion"*** hosts respective the container's build ingredients. The Algo/pre-processing logics are placed under e.g. ***"safety_gear_demo"*** & ***sample_classification*** to showcase a grouping of logically related entities otherwise it is not a mandatory directory layout.

  * ## *config.json*
    This file defines UDF specific configuration and other generic configs such as queue-depth, number-of-worker-thread etc. These generic configs can be added to overwrite any default of setting of VI and VA container. In order to know more about schema of defining these configs and its permissible values, kindly refer [UDF-README](../common/video/udfs/README.md) file. An example snippet would look as below:
    ```bash 
    {
        "encoding": {
            "level": 95,
            "type": "jpeg"
        },
        "max_jobs": 20,
        "max_workers": 4,
        "queue_size": 10,
        "udfs": [
            {
                "name": "safety_gear_demo",
                "type": "native",
                "device": "CPU",
                "model_xml": "safety_gear_demo/ref/frozen_inference_graph.xml",
                "model_bin": "safety_gear_demo/ref/frozen_inference_graph.bin"
            }
        ]
    }
    ```
  * ## *Dockerfile*
    This file defines the container build process and what all build time and runtime artifacts need to be copied to the container.
    In case of native(c++) UDF we need to describe the destination path to copy the code to native UDF and compilation instruction of the same. And in case of python we just need to copy the udf defining artifacts to proper destination location. Some code comments are given for describing important key values. 
    An example ***Dockerfile*** for C++ based UDF is pasted below:
    
    ```dockerfile
    ARG EIS_VERSION                                          <<<<This is to use latest version of VI & VA automatically instead of hardcoding a version
    ARG DOCKER_REGISTRY
    FROM ${DOCKER_REGISTRY}ia_video_analytics:$EIS_VERSION   <<<<<This container is based VA container
    LABEL description="C++ based Safety Gear UDF Image"

    WORKDIR ${GO_WORK_DIR}

    RUN /bin/bash -c "source /opt/intel/openvino/bin/setupvars.sh && \
        cd ./safetty_gear_demo && \
        rm -rf build && \
        mkdir build && \
        cd build && \
        cmake -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE} .. && \
        make && \
        make install"
    ```
    An Example of python based UDF's ***Dockerfile*** will looks as below:

    ```dockerfile
    ARG EIS_VERSION
    ARG DOCKER_REGISTRY
    FROM ${DOCKER_REGISTRY}ia_video_ingestion:$EIS_VERSION
    LABEL description="Multi-class clasifcation UDF Image"

    WORKDIR ${GO_WORK_DIR}

    # Added GO_WORK_DIR to python path as copying the UDF code block under GO_WORK_DIR
    # else user can add the path to which it copies its udf code.
    ENV PYTHONPATH ${PYTHONPATH}:${GO_WORK_DIR}

    # User can mention about all UDF directories here, both py and C++.
    COPY ./sample_classification ./sample_classification  <<<<< ./sample_classification is the destination here for UdfLoader to pick it at runtime. Additionally we have set the python path for UdfLoader to identify the algo artifacts properly.
    ```

  * ## *docker-compose.yml*
    The docker-compose.yml makes the custom UDF container to be managed by EIS infrastructure. The content for this file also defines how the UDF container will communicate with other containers via messagebus. Some of the important key-value pairs of this docker-compose.yml file is described below. The value of certain  keys can be altered based on the need. Below mentioned keys need to alter for a different UDF container. Import key-value combinations are described in the code comments with prescript of "<<<<<".

    ```yml
    services:
      python_multi_classification:      <<<<< Define the name of the container
        depends_on:
        - ia_video_ingestion            <<<<< If it is based on VA then it should be ia_video_ananlytics
        build:
          context: $PWD/../CustomUdfs/PyMultiClassificationIngestion                 <<<<< This path should be the relative path of container artifact
          dockerfile: $PWD/../CustomUdfs/PyMultiClassificationIngestion/Dockerfile   <<<<< Path to Dockerfile of the container
    -----snip-----
        image: ${DOCKER_REGISTRY}python_multi_classification:${EIS_VERSION} <<<<< Image Name
        container_name: python_multi_classification                         <<<<< container Name
        hostname: python_multi_classification                               <<<<< Docker Network Namespace defined container name
    -----snip----
        environment:
          AppName: "PyMultiClassificationIngestion"                                  <<<<< Application name which will be used by configuration server & for security purpose.
    ----snip------
          PubTopics: "py_analytic_result_strm"                              <<<<< Publisher topic of this container And sub topic can be added based on need
          py_analytic_result_strm_cfg: "zmq_ipc,${SOCKET_DIR}/"
          Server: "zmq_tcp,127.0.0.1:66018"
    -----snip------
        privileged: true
        devices:
          - "/dev/:/dev/"
        secrets:                                                             <<<<< In the PROD mode the "secret" section need to be defined as described.
          - ca_etcd
          - etcd_PyMultiClassification_cert
          - etcd_PyMultiClassification_key

    secrets:
      etcd_PyMultiClassification_cert:                                       <<<<< The cerificate mentioned below are created during provision step
        file: provision/Certificates/PyMultiClassificationIngestion/PyMultiClassification_client_certificate.pem
      etcd_PyMultiClassification_key:
        file: provision/Certificates/PyMultiClassificationIngestion/PyMultiClassification_client_key.pem
    ```
  * ## *UDF core-logic Directory*
    This directory need to have the Algo/pre-processing implementation which defines the necessary callbacks, IR files and other configurational files as per need. User can place them directly without having another directory level too, in that case the ***Dockerfile*** and ***docker-compose.yml*** should update the path accordingly.  User can find sample implementation in [custom sample UDF](../CustomUdfs) directory.

# **Build and deploy Process**
The build process is similar to the EIS's build and deploy process with some minor chnages. Please find the ordered steps for building and deploying the Custom UDFs.

  * As per EIS default scenario, the sample custom UDF containers are not mandatory containers to run, hence the eis_builder.py should run "video-streaming-all-udfs.yml". All the sample UDF containers are added in this example. Below code snnipet signifies the same.

    ```yml
    AppName:
    - VideoIngestion
    - VideoAnalytics
    - Visualizer
    - WebVisualizer
    - CustomUdfs/NativeSafetyGearAnalytics   <<<<< All lines from here added are customUDFs, User can define his own container directory here
    - CustomUdfs/NativeSafetyGearIngestion
    - CustomUdfs/PyMultiClassificationIngestion
    - CustomUdfs/PySafetyGearAnalytics
    - CustomUdfs/PySafetyGearIngestion
    ```
    Run the following command:
    ```bash
    cd <multi-repo cloned path>/IEdgeInsights/build/
    python3 eis_builder.py -f video-streaming-all-udfs.yml
    ```   
    **Note:** 
    It is not mandatory to keep the custom Udfs in the CustomUdfs directory, but user must change the video-streaming-all-udfs.yml file accordingly to point the right path accordingly. 
    Additionally if it is placed under ***IEdgeInsights*** directory then the eis_builder.py file automatically picks it to generate a consolidated [***eis-config.json***](../build/config/eis-config.json) and [***docker-compose.yml***](../build/docker-compose.yml) file.

  * After generation of consolidated [***eis-config.json***](../build/config/eis-config.json) and [***docker-compose.yml***](../build/docker-compose.yml) file, Run the below command to provision the UDF containers. As a pre-cautionary measure, User can cross check the afore-mentioned file to verify the sanity of the UDF specific config and service details.  
    ```bash
    cd <WORK_DIR_PATH>/IEdgeInsights/build/provision
    sudo ./provision_eis.sh  ../docker-compose.yml
    ```
  * Build and run the containers.
    ```bash
    docker-compose up --build -d
    ```

# *Sample UDFs Directory*
In the CustomUdfs directory, there are 5 sample UDfs implemented and they related asshown below. These samples are created to showcase different use case.

```bash
.
├── NativeSafetyGearAnalytics
├── NativeSafetyGearIngestion

├── PyMultiClassificationIngestion

├── PySafetyGearAnalytics
├── PySafetyGearIngestion

└── README.md

```
The [*NativeSafetyGearIngestion*](./NativeSafetyGearIngestion) container has used [dummy](../common/video/udfs/native/dummy) UDF which is defined in VideoIngestion container. user can define its own preprocessing UDF and add to config.json file to modify it. The results are posted to a eis-msgbus topic which is subscribed by [NativeSafetyGearAnalytics](./NativeSafetyGearAnalytics) containers. The configs can be seen in [docker-compose.yml](./NativeSafetyGearAnalytics/docker-compose.yml) file.

Refer [NativeSafetyGearIngestion-README](./NativeSafetyGearIngestion/README.md) for more information on the udf configs.
Refer [NativeSafetyGearAnalytics-README](./NativeSafetyGearAnalytics/README.md) for more information on the udf configs.

The [PySafetyGearIngestion](./PySafetyGearIngestion) & [PySafetyGearAnalytics](./PySafetyGearAnalytics) are the same use-case but showcases the python variant of ingestion and analytics UDF containers

Refer [PySafetyGearIngestion-README](./PySafetyGearIngestion/README.md) for more information on the udf configs.
Refer [PySafetyGearAnalytics-README](./PySafetyGearAnalytics/README.md) for more information on the udf configs.

The [PyMultiClassificationIngestion](./PyMultiClassificationIngestion) showcases a UDF wherein everything is performed inside VideoIngestion containers, hence this is something user executes when it doesn't want to involve the eis-msgbus to be involved for data transfer between containers thereby achieving faster processing of the pipeline.

Refer [PyMultiClassificationIngestion-README](./PyMultiClassificationIngestion/README.md) for more information on the udf configs.

**Notes**:
* It is not mandatory to have Ingestion containers for every analytics UDF, UDFs are connected to each other via MSGBUS topics. Hence we can always use stock VideoIngestion container as long as Custom Analytic UDF container can read and churn the data it receives.

* User shouldn't remove VI & VA containers before first time build of custom UDF as it will fail to build custom UDFs. Once these UDFs are functional user can always get rid of running VI & VA containers.