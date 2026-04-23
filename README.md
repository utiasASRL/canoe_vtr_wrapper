## Download canoe_vtr_wrapper
NOTE: this repo uses `boreas_vtr_wrapper` as a template. The code in this repository has been modified for using the CANOE dataset.

This package contains testing code for lidar and radar pipeline. Download it do your local filesystem.

```Bash
git clone git@github.com:utiasASRL/canoe_vtr_wrapper.git
cd canoe_vtr_wrapper
git submodule update --init --recursive
```

## Build canoe_vtr_wrapper Docker Image

This builds a image that has all dependencies installed.

```Bash
bash scripts/build_docker.sh
```

## Start the canoe_vtr_wrapper Docker container

This will either attach to a canoe_vtr_wrapper container that's already running or create a new container which gets deleted upon exit.

```Bash
bash scripts/run_docker.sh
```

FYI: This will always run `entrypoint.sh` which tries to source the python venv (see below). Also, the container expects CANOE sequences to be stored at `$VTRRDATA`. If your data is not stored here, then consider mounting a volume in the shell script like `-v host_path:container_path:ro`

## Build and Install VT&R3 + canoe_vtr_wrapper

Inside the container

```Bash
bash scripts/build_packages.sh
```

If this build is crashing your computer, consider changing `scripts/build_vtr3.sh` and `scripts/build_canoe_vtr_wrapper.sh` to `MAKEFLAGS="-j1"`

## Create a python venv to install pycanoe

Within the running container

```Bash
bash scripts/create_venv.sh
source scripts/setup_container.sh
```

# Running Experiments

## Visualization (Work in Progress)

First launch RVIZ for visualization:

```Bash
source /opt/ros/galactic/setup.bash               # source the ROS environment
ros2 run rviz2 rviz2 -d ${VTRSRC}/rviz/radar.rviz # launch rviz
```

Then in another terminal, launch `rqt_reconfigure` for control. Currently supported dynamic reconfigure parameters: `control_test.play` and `control_test.delay_millisec`

```Bash
source /opt/ros/galactic/setup.bash
ros2 run rqt_reconfigure rqt_reconfigure
```

## Odometry (Teach) and Localization (Repeat)

```Bash
export VTRRROOT=${VTRROOT}/canoe_vtr_wrapper # location of this repository CHANGE THIS!
export VTRRDATA=${VTRDATA}/canoe/sequences  # dataset location (where the canoe-xxxxx folders at) CHANGE THIS!
export VTRRRESULT=${VTRTEMP}/radar/canoe    # default result location
mkdir -p ${VTRRRESULT}
```

```Bash
source ${VTRRROOT}/src/install/setup.bash
# Choose a Teach (ODO_INPUT) and Repeat (LOC_INPUT) run from canoe dataset
ODO_INPUT=canoe-2025-08-20-16-07
LOC_INPUT=canoe-2025-08-21-11-31
```

Odometry:
```Bash
bash ${VTRRROOT}/src/vtr_testing_radar/script/test_odometry.sh ${ODO_INPUT}
bash ${VTRRROOT}/src/vtr_testing_radar/script/test_odometry_eval.sh ${ODO_INPUT}
```

Localization:
```Bash
bash ${VTRRROOT}/src/vtr_testing_radar/script/test_localization.sh ${ODO_INPUT} ${LOC_INPUT}
# Evaluation:
bash ${VTRRROOT}/src/vtr_testing_radar/script/test_localization_eval.sh ${ODO_INPUT}
```
## [License](./LICENSE)
