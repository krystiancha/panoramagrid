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

## Build the image

```bash
docker build -t krystianch/panoramagrid .
```

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
    --volume $(pwd)/ros:/root/catkin_ws/src/panoramagrid \
    --rm \
    --interactive \
    --tty \
    krystianch/panoramagrid

xhost -local:panoramagrid
```

## Download the image set (1.6 GB)

```bash
curl -o images.zip https://protecto-static.s3.eu-central-1.amazonaws.com/images.zip
```

And copy the image set to the running container:

```bash
docker cp images.zip panoramagrid:/usr/local/share/panoramagrid/images.zip
```

## Try some examples

```bash
# While in the container...

# Run the ROS demo
# After running this command, RViz will launch.
# In order to control the camera add an interactive marker.
# Displays -> Add -> By topic -> /simple_marker -> /update -> InteractiveMarkers
roslaunch panoramagrid demo.launch

# Preview an equirectangular projection as a perspective view
# (use your mouse to move the camera around)
equirectviewer -i /usr/local/share/panoramagrid/sample_equirectangular.jpg

# Preview a cubemap as a perspective view
cubemapviewer -i /usr/local/share/panoramagrid/sample_cubemap.jpg
```
