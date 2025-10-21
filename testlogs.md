# Tests being run by Alan

**Currently** I am working on running the `estimator_node.py` in the loop while a ROS bag is running.
I am doing this by attaching to the Docker container that has been built and trying to run the ROS node
while playing a ROS bag.

```bash
# need to run the following two commands in several windows/panes to have
# several instances inside the same docker container
make up # enter/attach to the docker container
source devel/setup.bash

# run each of these in a separate pane
roscore
rosbag play /data/spurdog/Summer_2025_Spurdog_Data/multi_agent_bags/lawncross_24JUl/lawncross_joined.py
rosrun spurdog_acomms estimator_node.py
```

**Things I have needed to install inside the Docker container to do this** are
both `apt` installable packages and the python bindings of CORA, which we need
to build ourselves

`apt` and `pip` installable packages
```bash
pip install attrs scipy gtsam
sudo apt install build-essential cmake-gui libeigen3-dev liblapack-dev libblas-dev libsuitesparse-dev -y
```

`cora` python bindings
```bash
mkdir -p /ws/src/multi-spurdog/cora/build/
cd /ws/src/multi-spurdog/cora/build/
cmake ..
make -j
../bindings/install_cora_python.sh # installs the built python bindings to the current python environment (inside docker container)
```