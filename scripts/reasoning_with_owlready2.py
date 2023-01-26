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
ocra_namespace = get_namespace("http://www.iri.upc.edu/groups/perception/OCRA/ont/ocra.owl#")
# extract imported ontologies of ocra (think of a better way to extract all the nested imported ontologies)
imported_ontologies = ocra_onto.imported_ontologies
for imported_ontology in imported_ontologies:
    ## print(imported_ontology)
    ## print(imported_ontology.base_iri)
  
    imported_ontologies_dict[imported_ontology.base_iri] = imported_ontology    

print(imported_ontologies_dict)

dul_onto = imported_ontologies_dict[onto_iris["dul"]]
dul_namespace = get_namespace("http://www.ontologydesignpatterns.org/ont/dul/DUL.owl#")

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

print("\n··· Syncronizing the reasoner...")
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

print("\n··· Participants of an instance of PlanAdaptation")
plan_adaptation_instance_participants = plan_adaptation_instances_list[0].hasParticipant
print(plan_adaptation_instance_participants)

# Creating a new instance of the class 'Agent'
kinova_robot_2 = dul_onto.Agent("Kinova_robot_2", namespace = application_onto)

# Adding a new participant to the Plan Adaptation
plan_adaptation_instances_list[0].hasParticipant.append(kinova_robot_2)

print("\n··· Participants of an instance of PlanAdaptation (after adding more)")
plan_adaptation_instance_participants = plan_adaptation_instances_list[0].hasParticipant
print(plan_adaptation_instance_participants)
# Learn more about properties here: https://owlready2.readthedocs.io/en/v0.37/properties.html

# Creating a new instance of the class 'Agent' but with a string (to simulate a received msg from the robot)
kinova_robot_3 = getattr(dul_onto, 'Agent')("Kinova_robot_3", namespace = application_onto)

# Adding a new participant to the Plan Adaptation with a string (to simulate a received msg from the robot)
getattr(plan_adaptation_instances_list[0], 'hasParticipant').append(kinova_robot_3)

print("\n··· Participants of an instance of PlanAdaptation (after adding more)")
plan_adaptation_instance_participants = plan_adaptation_instances_list[0].hasParticipant
print(plan_adaptation_instance_participants)
# Learn more about properties here: https://owlready2.readthedocs.io/en/v0.37/properties.html

# Performing SPARQL queries
print("\n··· Result of a SPARQL query about all the events and their participants")
events_and_participants_list = list(default_world.sparql("""
           SELECT ?x ?y
           { ?x DUL:hasParticipant ?y . }
    """))

""" NOTE ABOUT PREFIXES IN OWLREADY  

Owlready automatically creates prefixes from the last part of ontology IRI (without .owl extension), 
e.g. the ontology “http://test.org/onto.owl” with be automatically associated with the “onto:” prefix.

"""
for event_participant in events_and_participants_list:
    print(event_participant)

print("\n··· Result of a SPARQL query about all the instances of PlanAdaptation")
instances_of_plan_adaptation_list = list(default_world.sparql("""
           SELECT ?x
           { ?x rdf:type ocra:PlanAdaptation . }
    """))

""" NOTE ABOUT PREFIXES IN OWLREADY  

Owlready automatically creates prefixes from the last part of ontology IRI (without .owl extension), 
e.g. the ontology “http://test.org/onto.owl” with be automatically associated with the “onto:” prefix.

"""
for instance_of_plan_adaptation in instances_of_plan_adaptation_list:
    print(instance_of_plan_adaptation)


print("\n··· Result of a SPARQL query about the current capacity of the board")
current_board_capacity_list = list(default_world.sparql("""
           SELECT ?x
           { ocra_filling_a_tray_adaptation_full_compartment:RFID_board_current_capacity DUL:hasDataValue ?x . }
    """))

for current_board_capacity in current_board_capacity_list:
    print(current_board_capacity)

# Asserting new knowledge to the ontology using SPARQL queries (note that to update existing knowledge, we need to delete the current content)
new_value = 9
with application_onto:
    default_world.sparql("""
        DELETE { ocra_filling_a_tray_adaptation_full_compartment:RFID_board_current_capacity DUL:hasDataValue ?x . }
        INSERT { ocra_filling_a_tray_adaptation_full_compartment:RFID_board_current_capacity DUL:hasDataValue """+str(new_value)+""" . }
        WHERE   { ocra_filling_a_tray_adaptation_full_compartment:RFID_board_current_capacity DUL:hasDataValue ?x . }
    """)

print("\n··· Result of a SPARQL query about the current capacity of the board (after updating it)")
current_board_capacity_list = list(default_world.sparql("""
           SELECT ?x
           { ocra_filling_a_tray_adaptation_full_compartment:RFID_board_current_capacity DUL:hasDataValue ?x . }
    """))

for current_board_capacity in current_board_capacity_list:
    print(current_board_capacity)

# Asserting new knowledge to the ontology using SPARQL queries including Python variables values
new_value = 8
another_value = 7
with application_onto:
    default_world.sparql("""
        DELETE { ocra_filling_a_tray_adaptation_full_compartment:RFID_board_current_capacity DUL:hasDataValue ?x . }
        INSERT { ocra_filling_a_tray_adaptation_full_compartment:RFID_board_current_capacity DUL:hasDataValue ??2 . }
        WHERE   { ocra_filling_a_tray_adaptation_full_compartment:RFID_board_current_capacity DUL:hasDataValue ?x . }
    """, [new_value, another_value]) # if (??X = ??1), the query will be done with 'new_value', (??X = ??2), then 'another_value'

print("\n··· Result of a SPARQL query about the current capacity of the board (after updating it)")
current_board_capacity_list = list(default_world.sparql("""
           SELECT ?x
           { ocra_filling_a_tray_adaptation_full_compartment:RFID_board_current_capacity DUL:hasDataValue ?x . }
    """))

for current_board_capacity in current_board_capacity_list:
    print(current_board_capacity)


# Asserting new knowledge to the ontology using SPARQL queries (new instance)
print("\n··· Result of a SPARQL query about the current instances of event in which the Kinova_robot participates")
events_where_robot_participates_list = list(default_world.sparql("""
           SELECT ?x
           { ?x DUL:hasParticipant ocra_filling_a_tray_adaptation_full_compartment:Kinova_robot ;
                rdf:type DUL:Event. }
    """))

for event_where_robot_participates in events_where_robot_participates_list:
    print(event_where_robot_participates)

with application_onto:
    default_world.sparql("""
        INSERT { ?n rdfs:comment "new individual assertion" ;
                    DUL:hasParticipant ocra_filling_a_tray_adaptation_full_compartment:Kinova_robot . }
        WHERE   { BIND(NEWINSTANCEIRI(DUL:Event) AS ?n)  }
    """) 

print("\n··· Result of a SPARQL query about the current instances of event in which the Kinova_robot participates (after asserting a new one)")
events_where_robot_participates_list = list(default_world.sparql("""
           SELECT ?x
           { ?x DUL:hasParticipant ocra_filling_a_tray_adaptation_full_compartment:Kinova_robot ;
                rdf:type DUL:Event. }
    """))

for event_where_robot_participates in events_where_robot_participates_list:
    print(event_where_robot_participates)


# Asserting new knowledge to the ontology using SPARQL queries (new instance to test inference)
print("\n··· Result of a SPARQL query about the current instances of plan adaptation in which the Kinova_robot participates")
events_where_robot_participates_list = list(default_world.sparql("""
           SELECT ?x
           { ?x DUL:hasParticipant ocra_filling_a_tray_adaptation_full_compartment:Kinova_robot ;
                rdf:type ocra:PlanAdaptation. }
    """))

for event_where_robot_participates in events_where_robot_participates_list:
    print(event_where_robot_participates)

with application_onto:
    default_world.sparql("""
        INSERT { ?n rdfs:comment "new individual assertion" ;
                    DUL:hasParticipant ocra_filling_a_tray_adaptation_full_compartment:Kinova_robot ;
                    DUL:hasPart ocra_filling_a_tray_adaptation_full_compartment:Full_compartment_adaptation_final_plan_execution .
                 ocra_filling_a_tray_adaptation_full_compartment:Full_compartment_adaptation_inital_plan_execution DUL:isPartOf ?n . }
        WHERE   { BIND(NEWINSTANCEIRI(DUL:Event) AS ?n)  }
    """) 

print("\n··· Syncronizing the reasoner...")
sync_reasoner(application_onto)


print("\n··· Result of a SPARQL query about the current instances of plan adaptation in which the Kinova_robot participates (after asserting a new one)")
events_where_robot_participates_list = list(default_world.sparql("""
           SELECT ?x
           { ?x DUL:hasParticipant ocra_filling_a_tray_adaptation_full_compartment:Kinova_robot ;
                rdf:type ocra:PlanAdaptation. }
    """))

for event_where_robot_participates in events_where_robot_participates_list:
    print(event_where_robot_participates)