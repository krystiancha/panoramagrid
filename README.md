# Panoramagrid

**WORK IN PROGRESS**

A library for visualizing, manipulating and searching grids of panoramic images.

# Requirements

- [Boost](https://www.boost.org/)
- [OpenCV](https://opencv.org/)
- [GLFW](https://www.glfw.org/)
- Python 3 headers

# Demos

Docker is used as a platform to run demos.

## Run the container

```bash
xhost +local:panoramagrid

docker run \
    --name panoramagrid \
    --hostname panoramagrid \
    --env DISPLAY \
    --env QT_X11_NO_MITSHM=1 \
    --device=/dev/dri:/dev/dri \
    --volume /tmp/.X11-unix:/tmp/.X11-unix:rw \
    --volume ./ros:/root/catkin_ws/src/panoramagrid \
    --rm \
    --interactive \
    --tty \
    protecto/panoramagrid

xhost -local:panoramagrid
```

### Try some examples

```bash
# While in the container...

# Run the ROS demo
roslaunch panoramagrid demo.launch

# Preview an equirectangular projection as a perspective view
# (use your mouse to move the camera around)
equirectviewer -i /usr/local/share/panoramagrid/sample_equirectangular.jpg

# Preview a cubemap as a perspective view
cubemapviewer -i /usr/local/share/panoramagrid/sample_cubemap.jpg
```
