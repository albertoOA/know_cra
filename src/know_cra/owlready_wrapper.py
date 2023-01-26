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