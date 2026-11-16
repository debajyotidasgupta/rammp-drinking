# syntax=docker/dockerfile:1
# rammp_drinking as a RAMMP module image: the drinking engine (capability_engine.CapabilityEngine) and the rammp-drink
# CLI, on rammp_feeding's image -- its guarded arm, path checks, cameras, MediaPipe face and the engine contract, which
# this module uses as they are (the version pinned here). CPU only: SAM3 and cuRobo are their own nodes.
ARG FEEDING_IMAGE=debajyotidasgupta/rammp-feeding:0.5.4
FROM ${FEEDING_IMAGE}

WORKDIR /module_ws
COPY . /module_ws/src/rammp_drinking/
RUN . /opt/ros/humble/setup.sh \
    && . /ros2_ws/install/setup.sh \
    && . /module_ws/install/setup.sh \
    && colcon build --packages-select rammp_drinking \
    && rm -rf /module_ws/build /module_ws/log

# `ros2 launch` on Humble has no SIGTERM handler: as PID 1 it would ignore docker stop's SIGTERM until SIGKILL. On SIGINT
# it stops its node, which cancels the action in flight and lets the arm settle first.
STOPSIGNAL SIGINT
CMD ["ros2", "launch", "rammp_drinking", "drinking.launch.py"]
