# Requirements

## Expected Behavior
The idea for this list is what should happen for all circumstances. Any change in behavior must be document on this list.
>[current mode],[observed change],[old value],[new value],[expected change in state]

- Item 1

## Possible Failures
This list should document any behavior that should be a considered a failure, for example any condition that isn't obvious from the list of expected behaviors. Anything in this list should be able to be addressed above.

### Catastrophic Failures
- Battery depletes while drone is in the air
- Drone flies beyond operator visual range of operator
- Drone flies into ground
- Drone flies into obstacle
- Drone does not respond to commands from ground station or RC controller
- 

### Failures in effectiveness (unable to complete goals)
- Drone keeps looping over good terrain...not marking it as visited and never returning to auto
- Drone flies too high over terrain for good photos
- Drone flies too low over terrain for good photos
- Drone fails to return to launch at end of waypoints, waits for operator to command RTL
- Drone fails to save images of terrain
- 

