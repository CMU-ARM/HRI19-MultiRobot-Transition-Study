# HRI19: MultiRobot Transition Study

COPYRIGHT(C) 2019 - CMU - Code released under MIT.
Contact - Zhi - zhi.tan@ri.cmu.edu

The [`beep.wav`](hri19_multirobot_transition_study_mobile/res/audio/beep.wav) sound file used was downloaded from `http://soundbible.com` and original made by Mike Koenig who released it under Creative Commons Attribution 3.0. It was modified by Zhi Tan for the study and continued to be released under Creative Commons Attribution 3.0.

This repository contains the code that ran the user study described in HRI19's **From One to Another: How Robot-Robot Interaction Affects Users’ Perceptions Following a Transition Between Robots** by **Xiang Zhi Tan, Samantha Reig, Elizabeth J. Carter, Aaron Steinfeld**. Due to the experimental nature of the work, the code does not meet industry coding guidelines / best practices and would need modifications to work on different machines. We provided a brief guide and pointers on how the system work and requires hardwares.


## Brief guide on how it might run:
1. Clone all the Baxter depencies and `hri19_multirobot_transition_study_baxter` into a ROS Workspace that controls Baxter
1. Clone all the Mobile robot depencies and `hri19_multirobot_transition_study_mobile` into a ROS Workspace on the mobile robot.
1. Hard code the position of the mobile robot at different location (next to robot, outside, etc) in [hri19_multirobot_transition_study_mobile/scripts/run.py](hri19_multirobot_transition_study_mobile/scripts/run.py).
1. Start the code with the start script on both machines.
    * use [run.sh](hri19_multirobot_transition_study_baxter/run.sh) script for baxter
    * use `rosrun hri19_multirobot_transition_study_mobile run.py`
1. Debug why it isn't running :p

### Notes on Replication
The code would require some effort to run on a different system. However, you can review the interaction flow in the [res](hri19_multirobot_transition_study_baxter/res/) folder. Each conditions has its own script file in the format `condition-stationary_robot_behavior-information_transfer.yaml`, where `stationary_robot_behavior` and `information_transfer` are the conditions described in the paper (Note `i` means `direct`, we called it `inter robot communication` when the code was written).

The interaction starts with [initial_base_interaction.yaml](hri19_multirobot_transition_study_baxter/hri19_/res/initial_base_interaction.yaml) that ask the participant their destination and walking speed. This interaction is first  called from every condition file. The interaction then follows the dialog listed in their respective condition file.

The interaction files should give a complete picture of the stationary robot's behavior during the study.

### Notes on Code
The code uses an old version of our baxter scripting language. The scripting language used is **NO LONGER** the same used in our future work.

## Podi External Git Dependencies
* [multibot_relay](https://github.com/CMU-ARM/multi_robot_relay)

## Baxter External Git Dependencies
Some of the dependencies might have been abandoned or discontinued since 2018.
* [lab_polly_speech](https://github.com/CMU-ARM/lab_ros_common)
* [lab_baxter_common](https://github.com/CMU-ARM/lab_baxter_common)
* [lab_ros_speech_to_text](https://github.com/CMU-ARM/lab_ros_speech_to_text)
* [multibot_relay](https://github.com/CMU-ARM/multi_robot_relay)
* [snips_nlu_ros](https://github.com/CMU-ARM/snips_nlu_ros)
