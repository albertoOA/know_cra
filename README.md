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

### Python3 virtual environment configuration and dependencies to use rosplan together with any of the knowledge bases (rosprolog or owlready2)

In the package, we already provide a virtual environment but it was built for our computer and it will probably not work in yours. You can easily delete it and create and configure your own environment executing the following commands in a terminal.
```
cd <know_cra_folder>/python_environment
python3 -m venv know_cra_rosplan
source know_cra_rosplan/bin/activate

python3 -m pip install wheel catkin_pkg rospkg pyyaml future
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

Finally, one can run a knowledge base with all the knowledge stored in an episodic memory, *validation neem*, which is used during the validation presented in the article [1].

```
roslaunch know_cra map_cra_cs_filling_tray_neem.launch
```

Alternatively, one can also run a knowledge base with all the knowledge stored in another episodic memory, *contrastive_plans/validation neem*, which is used during the validation presented in the article [2].

```
roslaunch know_cra map_cra_cs_bringing_object_plan_disambiguation_with_recorded_neem.launch
```

### Running an owlready2-based knowledge base for collaborative robotics and adaptation
```
rosrun konw_cra reasoning_with_owlready2.py
```

### Running a planning domain-agnostic knowledge base for objective evaluation of narratives 
The idea here is to run a generic knowledge base in which knowledge about two different plans will be stored for later comparison and contrastive narration.  

The next example assumes that ROSPlan is already installed, following the instructions on the gihtub README: [https://github.com/KCL-Planning/ROSPlan](https://github.com/KCL-Planning/ROSPlan).

```
roslaunch know_cra map_cra_cs_generic.launch
```

Now it is time to generate a couple of plans for a target domain and problem. In order to have two different plans for the same problem, it will be necessary to run twice the launch file modifying the argument *'evaluation'*. When it is set to 'false', the planner will take the first plan that is found after 1 second of graph search, if it is set to 'true' the planner will search during 120 seconds. Very oftent, the generated plan after 120 seconds will be better, but note that it will not always be true. In any case, two plans will be generated and their knowledge asserted to the knowledge base, where they will be compared using logical reasoning rules written in Prolog. Remember, you need to run the launch with *'evaluation'* set to 'false', close the execution and run it again with the argument set to 'true'. 

```
roslaunch know_cra rosplan_cra_cs_generic.launch 
``` 



**[1]** A. Olivares-Alarcos, A. Andriella, S. Foix and G. Alenyà. Robot explanatory narratives of collaborative and adaptive experiences, 40th IEEE International Conference on Robotics and Automation (ICRA), 2023, London, United Kingdom (pp. 11964-11971).

**[2]** A. Olivares-Alarcos, S. Foix, J. Borràs, G. Canal and G. Alenyà. (2024). Ontological modeling and reasoning for comparison and contrastive narration of robot plans. In Proceedings of the 2024 International Conference on Autonomous Agents and Multiagent Systems (AAMAS), 2024, Auckland, New Zealand, to appear.
