#!/usr/bin/env python3
# coding=utf-8
# Author: Alberto Olivares-Alarcos <aolivares@iri.upc.edu>, Institut de Robòtica i Informàtica Industrial
import rospy
import sys
import roslib
from rosprolog_client import PrologException, Prolog


class ROSPrologWrapperForROSPlanCRA:
    def __init__(self):
        # Define service clients

        # Define varibles
        self.client_rosprolog_ = Prolog()

        self.plan_types_to_ontology_classes_dict_ = {
                "garment" : "ocra_cloth:'Garment'", 
                "garment-type" : "ocra_cloth:'GarmentType'",
                "pile" : "ocra_cloth:'GarmentPile'", 
                "robot" : "dul:'PhysicalAgent'",
                "human" : "dul:'PhysicalAgent'",
                "concept" : "dul:'Concept'",
                "entity" : "dul:'Entity'",
                "object" : "dul:'Object'",
                "social-object" : "dul:'SocialObject'",
                "physical-object" : "dul:'PhysicalObject'",
        }
        self.ontology_classes_to_plan_types_dict_ = {
                "ocra_cloth:'Garment'" : "garment", 
                "ocra_cloth:'GarmentType'" : "garment-type",
                "ocra_cloth:'GarmentPile'" : "pile", 
                "dul:'PhysicalAgent'" : "robot",
                "dul:'PhysicalAgent'" : "human",
                "dul:'Concept'" : "concept",
                "dul:'Entity'" : "entity",
                "dul:'Object'" : "object",
                "dul:'SocialObject'" : "social-object",
                "dul:'PhysicalObject'" : "physical-object",
        }
        self.inverse_ontology_relations_dict_ = {
            ""
        }
    
    def types_and_instances_dict_to_triples_list(self, types_with_instances_dict):
        rospy.loginfo(rospy.get_name() + ": Formatting domain plan types and their instances as triples to assert them to the ontology KB")

        triples_list = list()
        for k, v in types_with_instances_dict.items():
            for i in v:
                triple = list()
                triple.append(i)
                triple.append('rdf:type')
                triple.append(self.plan_types_to_ontology_classes_dict_[k])
                
                triples_list.append(triple)
        
        return triples_list
    
    def construct_query_text_for_single_triple_assertion(self, triple_subject, triple_relation, triple_object, add_final_dot, add_inverse_triple):
        rospy.loginfo(rospy.get_name() + ": Construct query text for single triple assertion")
        query_text = "kb_project(triple(" + triple_subject + ", " + triple_relation + ", " + triple_object +"))"
        if add_inverse_triple:
            if (triple_relation != "rdf:type"):
                query_text = query_text + ", " + "kb_project(triple(" + triple_object + ", " + \
                    self.inverse_ontology_relations_dict_[triple_relation]+ ", " + triple_subject + "))"
            else: 
                pass
        else: 
            pass
        if add_final_dot:
            query_text = query_text + "."
        else:
            pass

        query_text = query_text.replace('-','_') # ontology does not like '-'

        return query_text

    def construct_query_text_for_multiple_triples_assertion(self, triples, add_inverse_triple):
        rospy.loginfo(rospy.get_name() + ": Construct query text for multiple triples assertion")

        query_text = self.construct_query_text_for_single_triple_assertion(triples[0][0], triples[0][1], triples[0][2], False, add_inverse_triple) 

        for i in range(1, len(triples)):
            query_text = query_text + ", " + \
                self.construct_query_text_for_single_triple_assertion(triples[i][0], triples[i][1], triples[i][2], False, add_inverse_triple) 
        
        query_text = query_text + "."

        return query_text

    def rosprolog_assertion_query(self, query_text):
        rospy.loginfo(rospy.get_name() + ": New query to assert ontological knowledge")

        # query the knowldge base
        query = self.client_rosprolog_.query(query_text)
        query_solutions = list()
        query.finish()

