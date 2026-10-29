# Devcontainer

ROS 2 Jazzy (`ros:jazzy-ros-base`) with Cyclone DDS. The repo is bind-mounted at
`/robotarm_core`, so colcon's `build/`, `install/` and `log/` live on the host.

## Dependencies

System dependencies come from the `package.xml` files under `src/` via `rosdep`.
A first build stage keeps only the `package.xml` and `*_IGNORE` files, so the rosdep
layer is rebuilt only when a dependency declaration changes, not on code edits.
Requires BuildKit (`docker buildx version`).

To add a dependency, declare it in the right `package.xml`, then rebuild the container
or run inside it:

```bash
rosdep install --from-paths src --ignore-src -y
```

Tools that no package declares (middleware, numpy) are installed directly in the `Dockerfile`.

## Runtime

- `--network=host`, `--ipc=host` for DDS; `--privileged` for USB/serial devices
- X11 forwarding for RViz/Qt (`xhost +local:docker` runs on the host before start)
