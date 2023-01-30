#!/usr/bin/env python3
# coding=utf-8
# Author: Alberto Olivares Alarcos <aolivares@iri.upc.edu>, Institut de Robòtica i Informàtica Industrial, CSIC-UPC

from owlready2 import *

class OWLreadyWrapper:
    def __init__(self):
        # variables
        self.loaded_ontology_ = None

    def get_ontology_wp(self, ont_string):
        self.loaded_ontology_ = get_ontology(ont_string).load()

    def sync_reasoner_wp(self):
        sync_reasoner(self.loaded_ontology_)

    def sparql_insert_delete_query_wp(self, query_string):
        with self.loaded_ontology_:
            default_world.sparql(query_string)

    def sparql_select_query_wp(self, query_string):
        with self.loaded_ontology_:
            answer_list = list(default_world.sparql(query_string))

            answer_dict = self.build_dictionary_from_sparql_answer(query_string, answer_list)

        return answer_dict

    def build_dictionary_from_sparql_answer(self, query_string, answer_list):
        # extract targets in query
        sub1 = "SELECT " # note the space after 'SELECT' 
        sub2 = "{"

        idx1 = query_string.index(sub1)
        idx2 = query_string.index(sub2)

        all_targets_str = query_string[idx1 + len(sub1) + 1 : idx2].strip()
        all_targets_str = ''.join(all_targets_str.split()) # remove all whitespaces
        all_targets_list = all_targets_str.split("?")

        # build dictionary
        answer_dict = dict()
        for j in range(0, len(all_targets_list)):
            answer_dict[all_targets_list[j]] = list()
            for answer in answer_list:
                answer_dict[all_targets_list[j]].append(answer[j])
        
        return answer_dict