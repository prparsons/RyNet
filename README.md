# RyNet
Drone search project. Navigate by segmentation network (looking for proper terrain), detection network running simultaneously (looking for the actual target, and collect more training data). Eventual end goal is to allow some transfer learning in flight if detection network finds targets, allowing it to improve accuracy in specific terrains. Running software on an nVidia Jetson, connected via UART to an ardupilot drone controller (CubePilot).

General "how it works": you program a route, and start the drone following that route in auto. The app will then switch it to guided when the segmentation network detects the desired category, and keep going so long as desired category is in frame. Once there's nothing else new in frame, it will remove any points it "visited" while in guided mode from the route, then re-arrange remaining waypoints for the shortest path possible before switching back to auto. Currently is working (camera in portrain mode), testing with dronekit's simulator as well as live flights without commanding drone (python script --livetest).

The python DroneKit library was the most complete library I found, so there's a python script that communicates directly with the drone. It listens via UDP for the c++ program that does the actual work. Since the drone will have wifi, implemented a simple hmac based authentication scheme for the UDP packets (also provides verification of message correctness/completeness).

Building/testing
It "should" be able to be built in windows via WSL. This is what I can remember from looking at the little nvidia jetson nano.
<ul>
  <li>
    Intel Realsense library
    <ul>
      <li>https://github.com/IntelRealSense/librealsense/tree/v2.39.0</li>
    </ul>
  </li>
  <li>
    CMake
    <ul>
      <li>
        Not certain what needed it, but I manually built and installed CMake 3.13.
      </li>
    </ul>
  <li>
    JetsonInference
    <ul>
      <li>
        Plan is to remove that, and directly run models via TensorRT.
      </li>
      <li>https://github.com/dusty-nv/jetson-inference</li>
    </ul>
  </li>
  <li>
    OpenCV2
    <ul>
      <li>?</li>
      <li>Needs/uses gstreamer</li>
    </ul>
  </li>
  <li>
    Dronekit
    <ul>
      <li>Python library</li>
    </ul>
  </li>
</ul>

This can be tested using Mission Planner's built in simulator. https://ardupilot.org/planner/ Should be enough command line options to disable the hardware that doesn't exist, point the python script at the simulator (directions in its comments). Can load a video file to simulate the RGB camera input.

https://github.com/VainF/DeepLabV3Plus-Pytorch
That's the project I was planning to use for training the terrain scanning network (image segmentation). Drone is running a Jetson Xavier NX, 8gb ram for whole system. So inference models need to fit in it. As for training, I have a 16gb card...memory is the limiting factor.

I haven't really come up with what to use for detection...jetson inference has some decent models, and it might be worth just continuing to use that out of simplicity?

release build:
$ cmake -DCMAKE_BUILD_TYPE=Release ..