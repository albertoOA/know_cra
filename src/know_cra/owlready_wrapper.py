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