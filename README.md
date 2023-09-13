# 2023 FTC Season "CENTERSTAGE"
# Team 14623(Wolfpack) and 22291(Bounty Hunters)
## Overview
This code utilizes FTCLib which can be found at <https://docs.ftclib.org/ftclib/>


## Robot
The code for this robot utilizes a 3 wheel killough drive setup.
The following subsystems will be coded during the season.
- Arm
- Launcher
- Climber
- Drivetrain
Only the drivetrain subsystem has been implemented.

Vision examples for reading Apriltags has been implemented for testing purposes.

## Season coding expectations
### Autonomous
- Team Prop object detection to place pixel on correct spike mark
- Team Prop AprilTag detection to place pixel on correct backdrop location
- Park in backstage
- Above routine done if setup on left or right of rigging
- Travel to get more pixel from other end of field if time permits

### Driver Controlled
- Drive in field oriented mode with holonomic drivetrain (3 wheel Kiwi/Killough)
- Detect AprilTag or backstage tape to stop robot from accidentally hitting the backdrop
- Button commands to point the chassis in a direction with a PID loop
- Launcher commands
- Climber commands
- Arm rotator, extension, and grabbing commands








