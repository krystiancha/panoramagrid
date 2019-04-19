# Panoramagrid

## Docker

### Run the container

```bash
# Enable GUI in a container, may not be needed depending on your configuration
xhost +local:panoramagrid  

docker run \
  --device /dev/dri:/dev/dri \
  --env DISPLAY \
  --hostname panoramagrid \
  --interactive \
  --volume /tmp/.X11-unix:/tmp/.X11-unix \
  --tty \
  --user developer \
  protecto/panoramagrid
  
xhost -local:panoramagrid
```

### Try some examples

```bash
# While in the container...

# Preview an equirectangular projection as a perspective view
# (use your mouse to move the camera around)
equirectviewer -i /usr/local/share/panoramagrid/sample_equirectangular.jpg

# Preview a cubemap as a perspective view
cubemapviewer -i /usr/local/share/panoramagrid/sample_cubemap.jpg
```
