#!/usr/bin/env python3
# coding=utf-8
# Author: Alberto Olivares-Alarcos <aolivares@iri.upc.edu>, Institut de Robòtica i Informàtica Industrial
import rospy
import sys
import roslib
from datetime import datetime
from rosprolog_client import PrologException, Prolog


class ROSPrologWrapperForROSPlanCRA:
    def __init__(self):
        # Define service clients

        # Define varibles
        self.client_rosprolog_ = Prolog()
        self.semantic_map_namespace_ = "map_piling_cloth"

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

        self.inverse_ontology_relations_dict_ = self.get_ontology_property_and_inverse_dict()

        self.unitary_plan_predicates_to_ontology_classes_dict_ = {
            "graspable" : "dul:'Role'", 
            "free-to-manipulate" : "dul:'Role'",
            "piled" : "dul:'Role'",
            "supported" : "dul:'Role'",
            "lifted" : "dul:'Role'",
            "folded" : "dul:'Role'",
            "unfolded" : "dul:'Role'",
            #"current-number-of-garments-on-pile" : "dul:'Quality'",
        }

        self.unitary_plan_predicates_to_ontology_relations_dict_ = {
            "graspable" : "dul:'isRoleOf'", 
            "free-to-manipulate" : "dul:'isRoleOf'",
            "piled" : "dul:'isRoleOf'",
            "supported" : "dul:'isRoleOf'",
            "lifted" : "dul:'isRoleOf'",
            "folded" : "dul:'isRoleOf'",
            "unfolded" : "dul:'isRoleOf'",
            #"current-number-of-garments-on-pile" : "dul:'isQualityOf'",
        }

        self.binary_plan_predicates_to_ontology_relations_dict_ =  {
            "grasped-by" : "ocra_common:'isGraspedBy'",
            "on-pile" : "dul:'hasLocation'",
            'is-classified-by' : "dul:'isClassifiedBy'",
            'has-quality' : "dul:'hasQuality'",
        }

    
    def types_and_instances_dict_to_triples_list(self, types_with_instances_dict):
        rospy.loginfo(rospy.get_name() + ": Formatting domain plan types and their instances as triples to assert them to the ontology KB")

        triples_list = list()
        for k, v in types_with_instances_dict.items():
            for i in v:
                triple = list()
                triple.append(self.semantic_map_namespace_ + ":'" + i.replace('-','_') + "'") # ontology does not like '-'
                triple.append("rdf:'type'")
                triple.append(self.plan_types_to_ontology_classes_dict_[k])
                
                triples_list.append(triple)
        
        return triples_list

    def subgoals_dict_to_triples_list(self, subgoals_dict):
        rospy.loginfo(rospy.get_name() + ": Formatting the problem goal as a set of triples to assert them to the ontology KB")

        triples_list = list()
        goal_id = "problem_goal_" + str(datetime.utcnow()).replace(" ", "_") + "-UTC"
        triples_list.append([self.semantic_map_namespace_ + ":'" + goal_id + "'", "rdf:'type'", "dul:'Goal'"])

        for k, v in subgoals_dict.items():
            triple_st = list()
            statement_id = goal_id + "_" + k
            triple_st.append(self.semantic_map_namespace_ + ":'" + statement_id + "'")
            triple_st.append("rdf:'type'")
            triple_st.append("rdf:'Statement'")
            triples_list.append(triple_st)

            triple_st_subject = list()
            triple_st_predicate = list()
            triple_st_object = list()
            triple_st_reified = list()
            triple_concept_individual = list()
            if len(v) == 2: # unitary planning predicate
                triple_st_subject.append(self.semantic_map_namespace_ + ":'" + statement_id + "'")
                triple_st_subject.append("rdf:'subject'")
                triple_st_subject.append(self.semantic_map_namespace_ + ":'" + v[0].replace('-','_') + "'") # (concept) instance
                triples_list.append(triple_st_subject)

                triple_st_predicate.append(self.semantic_map_namespace_ + ":'" + statement_id + "'")
                triple_st_predicate.append("rdf:'predicate'")
                triple_st_predicate.append(self.unitary_plan_predicates_to_ontology_relations_dict_[v[0]]) 
                triples_list.append(triple_st_predicate)

                triple_st_object.append(self.semantic_map_namespace_ + ":'" + statement_id + "'")
                triple_st_object.append("rdf:'object'")
                triple_st_object.append(self.semantic_map_namespace_ + ":'" + v[1].replace('-','_') + "'") # problem (object) instance
                triples_list.append(triple_st_object)

                triple_concept_individual.append(self.semantic_map_namespace_ + ":'" + v[0].replace('-','_') + "'")
                triple_concept_individual.append("rdf:'type'")
                triple_concept_individual.append(self.unitary_plan_predicates_to_ontology_classes_dict_[v[0]])
                triples_list.append(triple_concept_individual)
            elif len(v) == 3: 
                triple_st_subject.append(self.semantic_map_namespace_ + ":'" + statement_id + "'")
                triple_st_subject.append("rdf:'subject'")
                triple_st_subject.append(self.semantic_map_namespace_ + ":'" + v[1].replace('-','_') + "'") 
                triples_list.append(triple_st_subject)

                triple_st_predicate.append(self.semantic_map_namespace_ + ":'" + statement_id + "'")
                triple_st_predicate.append("rdf:'predicate'")
                triple_st_predicate.append(self.binary_plan_predicates_to_ontology_relations_dict_[v[0]]) 
                triples_list.append(triple_st_predicate)

                triple_st_object.append(self.semantic_map_namespace_ + ":'" + statement_id + "'")
                triple_st_object.append("rdf:'object'")
                triple_st_object.append(self.semantic_map_namespace_ + ":'" + v[2].replace('-','_') + "'") 
                triples_list.append(triple_st_object)
            else:
                rospy.logerr(rospy.get_name() + ": Unexpected planning predicate length")

            triple_st_description = list()
            triple_st_description.append(self.semantic_map_namespace_ + ":'" + goal_id + "'")
            triple_st_description.append("ocra_common:'describesReifiedStatement'")
            triple_st_description.append(self.semantic_map_namespace_ + ":'" + statement_id + "'")
            triples_list.append(triple_st_description)

        return triples_list

    
    def construct_query_text_for_single_triple_assertion(self, triple_subject, triple_relation, triple_object, add_final_dot, add_inverse_triple):
        ## rospy.loginfo(rospy.get_name() + ": Construct query text for single triple assertion")
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

        ## query_text = query_text.replace('-','_') # ontology does not like '-'

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

    def get_ontology_property_and_inverse_dict(self):
        ont_property_inverse_dict = dict()

        query = self.client_rosprolog_.query("kb_call(triple(S, owl:'inverseOf', O))")

        for solution in query.solutions():
            subj_ = solution['S'].split('#')[-1]
            obj_ = solution['O'].split('#')[-1]

            ont_property_inverse_dict[subj_] = obj_
            ont_property_inverse_dict[obj_] = subj_

        query.finish()

        return ont_property_inverse_dict

