# KNOW-CRA
ROS package to launch a knowledge base to represent and reason with knowledge about collaborative robotics and adaptation.


### Dependencies

```
sudo apt install swi-prolog libjson-glib-dev

sudo apt install python-rdflib

wstool set know_cra --git https://github.com/albertoOA/know_cra.git

wstool update know_cra

wstool merge know_cra/rosinstall/know_cra.rosinstall

wstool update 
```

### Building

```
roscd; cd ..

catkin_make -DCATKIN_WHITELIST_PACKAGES="know_cra;comp_spatial;knowrob_common;rosprolog;json_prolog_msgs;rosowl;genowl"
```

### Running a knowledge base for collaborative robotics and adaptation
First, we can run a knowledge base with the basic knowledge for a specific use case in which a robot and a human collaborate to fill the compartments of a tray with tokens.

```
roslaunch know_cra map_cra_cs_filling_tray.launch
```

Finally, we can run a knowledge base with all the knowledge stored in an episodic memory, *validation neem*, which is used during the validation presented in our article [1].

```
roslaunch know_cra map_cra_cs_filling_tray_neem.launch
```


**[1]** A. Olivares-Alarcos, A. Andriella, S. Foix and G. Alenyà. Robot explanatory narratives of collaborative and adaptive experiences, 40th IEEE International Conference on Robotics and Automation (ICRA), 2023, London, United Kingdom, submitted.
