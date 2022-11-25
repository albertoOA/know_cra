#!/usr/bin/env python3

"""
What is this code? 
    - simple example to load OCRA and do reasoning over it using owlready2. Note 
    that in this case, we will use an application ontology containing knowledge 
    about a plan adaptation during a collaborative task of filling a tray. The 
    human fills the tray's compartment that the robot aimed to fill and the robot
    adpapts its plan. 
"""

## import os # alternative to avoid using ROS
import rospkg
from owlready2 import *

# Beginning global variables
onto_namespaces = {
    "http://www.iri.upc.edu/groups/perception/OCRA/ont/ocra.owl#" : "ocra",
    "http://www.ease-crc.org/ont/SOMA.owl#" : "soma", 
    "http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#" : "dul"
}

onto_iris = {
    "ocra" : "http://www.iri.upc.edu/groups/perception/OCRA/ont/ocra.owl#",
    "soma" : "http://www.ease-crc.org/ont/SOMA.owl#", 
    "dul" : "http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#"
}

# Getting the package path
rospack = rospkg.RosPack()
script_directory = rospack.get_path('know_cra')
## script_directory = os.path.realpath(os.path.dirname(__file__)) #  alternative to avoid using ROS
## print(script_directory)
# end global variables

# Loading an ontology
# - with URI
ocra_onto_with_uri = get_ontology("http://www.iri.upc.edu/groups/perception/OCRA/ont/ocra.owl").load() # ocra.owl / ocra_filling_a_tray.owl
# - with path file
application_onto = get_ontology("file://"+script_directory+"/owl/ont/ocra_filling_a_tray_adaptation_full_compartment.owl").load()
## application_onto = get_ontology("file://"+script_directory+"/../owl/ont/ocra_filling_a_tray_adaptation_full_compartment.owl").load()  # alternative to avoid using ROS

print("\n··· Classes")
application_onto_classes_list = list(application_onto.classes())
print(application_onto_classes_list)

print("\n··· Individuals (of the previous classes)")
for app_onto_class in application_onto_classes_list:
    print(app_onto_class.instances())
    continue 

print("\n··· Individuals (defined within the loaded ontology)")
application_onto_individuals_list = list(application_onto.individuals())
print(application_onto_individuals_list)

print("\n··· OCRA's IRI")
print(application_onto.base_iri)

print("\n··· Imported ontologies")
imported_ontologies = application_onto.imported_ontologies
imported_ontologies_dict = dict()
for imported_ontology in imported_ontologies:
    ## print(imported_ontology)
    ## print(imported_ontology.base_iri)
    
    imported_ontology_classes_list = list(imported_ontology.classes())
    ## print(imported_ontology_classes_list)
  
    imported_ontologies_dict[imported_ontology.base_iri] = imported_ontology    

## print(imported_ontologies_dict)


ocra_onto = imported_ontologies_dict[onto_iris["ocra"]]
# extract imported ontologies of ocra (think of a better way to extract all the nested imported ontologies)
imported_ontologies = ocra_onto.imported_ontologies
for imported_ontology in imported_ontologies:
    ## print(imported_ontology)
    ## print(imported_ontology.base_iri)
  
    imported_ontologies_dict[imported_ontology.base_iri] = imported_ontology    

print(imported_ontologies_dict)

dul_onto = imported_ontologies_dict[onto_iris["dul"]]

print("\n··· Classes of an imported ontology (OCRA)")
ocra_onto_classes_list = list(ocra_onto.classes())
print(ocra_onto_classes_list)

print("\n··· Instances of an imported ontology (OCRA)")
ocra_onto_instances_list = list(ocra_onto.individuals())
print(ocra_onto_instances_list)

print("\n··· Instances of PlanAdaptation of an imported ontology (OCRA)")
# The next line only includes the actual instances (one may try with 'CollaborationPlace'
# because there are no instances of PlanAdaptation at this point)
plan_adaptation_instances_list = list(ocra_onto.PlanAdaptation.instances())
print(plan_adaptation_instances_list)

# The next line also includes the ocra.PlanAdaptation and it needs a target search iri (e.g.
# application_onto). Note that one may try with 'CollaborationPlace' to check this
print(application_onto.search(is_a = ocra_onto.PlanAdaptation)) 

## sync_reasoner()
## sync_reasoner(ocra_onto)
sync_reasoner(application_onto)

print("\n··· Instances of PlanAdaptation after reasoning (OCRA)")
plan_adaptation_instances_list = list(ocra_onto.PlanAdaptation.instances())
print(plan_adaptation_instances_list)
print(application_onto.search(is_a = ocra_onto.PlanAdaptation))

print("\n··· Inconsistent classes after reasoning")
print(list(default_world.inconsistent_classes())) # print inconsistent classes

print("\n··· General axiomatization of a class (PlanAdaptation)")
subclass_of_axioms_PlanAdaptation = ocra_onto.PlanAdaptation.is_a
print(subclass_of_axioms_PlanAdaptation)

print("\n··· Equivalent axiomatization of a class (PlanAdaptation)")
equivalent_to_axioms_PlanAdaptation = ocra_onto.PlanAdaptation.equivalent_to
print(equivalent_to_axioms_PlanAdaptation)

print("\n··· Properties of an instance of PlanAdaptation")
plan_adaptation_instance_properties_list = plan_adaptation_instances_list[0].get_properties() # also get_inverse_properties()
print(plan_adaptation_instance_properties_list)

print("\n··· Properties of an instance of PlanAdaptation")
plan_adaptation_instance_participants = plan_adaptation_instances_list[0].hasParticipant
print(plan_adaptation_instance_participants)

# Creating a new instance of the class 'Agent'
kinova_robot_2 = dul_onto.Agent("Kinova_robot_2", namespace = application_onto)

# Adding a new participant to the Plan Adaptation
plan_adaptation_instances_list[0].hasParticipant.append(kinova_robot_2)

print("\n··· Properties of an instance of PlanAdaptation (after adding more)")
plan_adaptation_instance_participants = plan_adaptation_instances_list[0].hasParticipant
print(plan_adaptation_instance_participants)
# Learn more about properties here: https://owlready2.readthedocs.io/en/v0.37/properties.html
