#!/usr/bin/env bash
# Everything a non-interactive shell needs to run the simulation. Source it, do not execute it:
#
#     source ~/airmouse_ws/scripts/env.sh
#
# ~/.bashrc sets all of this too, but only for INTERACTIVE shells - so a script started over ssh,
# from cron, or from a `wsl.exe -- bash -lc ...` call gets none of it. That failure is quiet and
# confusing: Gazebo cannot resolve practice_6x6.sdf without GZ_SIM_RESOURCE_PATH, so it tries to
# download the name from Fuel, fails, and exits, leaving SITL and MAVROS running with no simulator
# and every topic dead. Keep this file in step with the airmouse block in ~/.bashrc.
source /opt/ros/jazzy/setup.bash
if [ -f "$HOME/airmouse_ws/install/setup.bash" ]; then
    source "$HOME/airmouse_ws/install/setup.bash"
fi

export GZ_VERSION=harmonic
export GZ_SIM_SYSTEM_PLUGIN_PATH=$HOME/ardupilot_gazebo/build:${GZ_SIM_SYSTEM_PLUGIN_PATH:-}
export GZ_SIM_RESOURCE_PATH=$HOME/airmouse_ws/sim/models:$HOME/airmouse_ws/sim/worlds:$HOME/ardupilot_gazebo/models:$HOME/ardupilot_gazebo/worlds:${GZ_SIM_RESOURCE_PATH:-}

# GPU rendering through WSL; harmless when running headless
export GALLIUM_DRIVER=${GALLIUM_DRIVER:-d3d12}
export MESA_D3D12_DEFAULT_ADAPTER_NAME=${MESA_D3D12_DEFAULT_ADAPTER_NAME:-NVIDIA}
