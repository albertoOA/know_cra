# KNOW-CRA
ROS package to use a knowledge base to represent and reason with knowledge about collaborative robotics and adaptation. This package allows to run the knowledge base using two different tools: *rosprolog* and *owlready2*. The first one includes features to be used within ROS (e.g., one can query the knowledge base calling a ROS service). The second option allows to use the ontology without ROS, just as we could use other OWL apis. 


### Dependencies to use a rosprolog-based knowledge base

```
sudo apt install swi-prolog libjson-glib-dev

sudo apt install python-rdflib

wstool set know_cra --git https://github.com/albertoOA/know_cra.git

wstool update know_cra

wstool merge know_cra/rosinstall/know_cra.rosinstall

wstool update 
```

### Python3 virtual environment configuration and dependencies to use an owlready2-based knowledge base

In the package, we already provide a virtual environment but it was built for our computer and it will probably not work in yours. You can easily delete it and create and configure your own environment executing the following commands in a terminal.
```
cd <know_cra_folder>/python_environment
python3 -m venv know_cra_owlready2
source know_cra_owlready2/bin/activate

python3 -m pip install cython owlready2 catkin_pkg rospkg pyyaml future
```

### Building

```
roscd; cd ..

catkin_make -DCATKIN_WHITELIST_PACKAGES="know_cra;comp_spatial;knowrob_common;rosprolog;json_prolog_msgs;rosowl;genowl"
```

### Running a rosprolog-based knowledge base for collaborative robotics and adaptation
First, we can run a knowledge base with the basic knowledge for a specific use case in which a robot and a human collaborate to fill the compartments of a tray with tokens.

```
roslaunch know_cra map_cra_cs_filling_tray.launch
```

Finally, we can run a knowledge base with all the knowledge stored in an episodic memory, *validation neem*, which is used during the validation presented in our article [1].

```
roslaunch know_cra map_cra_cs_filling_tray_neem.launch
```

### Running an owlready2-based knowledge base for collaborative robotics and adaptation
```
rosrun know_cra reasoning_with_owlready2.py
```

**[1]** A. Olivares-Alarcos, A. Andriella, S. Foix and G. Alenyà. Robot explanatory narratives of collaborative and adaptive experiences, 40th IEEE International Conference on Robotics and Automation (ICRA), 2023, London, United Kingdom, to appear.
