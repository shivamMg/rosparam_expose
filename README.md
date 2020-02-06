## rosparam_expose

Exposes ROS1 Parameter Server operations as ROS Services

### Build and Run

```
mkdir -p catkin_ws/src
cd catkin_ws
git clone https://github.com/shivamMg/rosparam_expose src/rosparam_expose
rosdep install --from-paths src/rosparam_expose
catkin_make
roslaunch rosparam_expose server.launch  # start server

# From a separate terminal
. catkin_ws/devel/setup.bash
rosservice list  # should show all services under /rosparam_expose/
rosservice call /rosparam_expose/set foo bar
rosservice call /rosparam_expose/get foo  # should print: value: "bar"

# rosparam-like CLI to call rosparam_expose services
# create a yaml
cat > load.yaml <<EOM
a: b
c: [1, 2]
d: false
EOM

rosrun rosparam_expose client load load.yaml /example
rosrun rosparam_expose client dump dump.yaml /example
cat dump.yaml  # should have same contents as load.yaml

# see example.launch for <rosparam/> tag examples
cat $(rospack find rosparam_expose)/launch/example.launch
```

### Try it on rapyuta.io

<a href="https://console.rapyuta.io/catalog?uo=1&link=https%3A%2F%2Fraw.githubusercontent.com%2FshivamMg%2Frosparam_expose%2Fmaster%2Fio_manifests%2Fclient-server-example.json">
  <img src="https://storage.googleapis.com/artifacts.rapyuta.io/images/import-package-button.svg?v0" width="224" height="37" />
</a>
