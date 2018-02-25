#!/bin/bash

rosservice call /phoebe_gripper_request "{angle: 0.44, distance: 0, id: 2, duration: 1.0}" # open
#rosservice call /phoebe_gripper_request "{angle: -0.38, distance: 0, id: 2, duration: 1.0}" # closed
