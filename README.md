# RyNet
Drone search project. Navigate by segmentation network (looking for proper terrain), detection network running simultaneously (looking for the actual target, and collect more training data). Eventual end goal is to allow some transfer learning in flight if detection network finds targets, allowing it to improve accuracy in specific terrains.

Currently is working (camera in portrain mode), testing with dronekit's simulator. Needs proper exception handling before live flight, as well as hardware failsafe to disconnect jetson from flight controller. Python server script needs creation (current .py file is just a notebook used to verify everything works)
