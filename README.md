# OASIS Kinect2 for ROS 2

This repo is a ROS 2 fork of the [Institute for Artificial Intelligence - University of Bremen Kinect2 driver](https://github.com/code-iai/iai_kinect2).

It is used by the [OASIS](https://github.com/eigendude/OASIS) smarthome OS.

The libfreenec2 bridge and depth registration are tested and working. The calibration and viewer packages have only partially been ported.

## Dependencies

All OASIS dependencies are managed by the [oasis_tooling](https://github.com/eigendude/OASIS/tree/main/oasis_tooling) package.

The specific versions are defined in the file [depends.repos](https://github.com/eigendude/OASIS/blob/main/oasis_tooling/config/depends.repos). The dependencies for specifically Kinect 2 functionality are:


```
repositories:
  # Requires: libturbojpeg0-dev libusb-1.0-0-dev ocl-icd-opencl-dev
  depends/libfreenect2:
    type: git
    url: https://github.com/OpenKinect/libfreenect2.git
    version: master
  # Requires: default-jdk libudev-dev libusb-1.0-0-dev
  depends/OpenNI2:
    type: git
    url: https://github.com/structureio/OpenNI2.git
    version: master
  ros-perception/image_pipeline:
    type: git
    url: https://github.com/ros-perception/image_pipeline.git
    version: rolling
  ros-perception/image_transport_plugins:
    type: git
    url: https://github.com/ros-perception/image_transport_plugins.git
    version: rolling
```

### Installation, GPU acceleration and Citations

The original README can be found at: https://github.com/code-iai/iai_kinect2
