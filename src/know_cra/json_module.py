#!/usr/bin/env python3
# coding=utf-8
# Author: Alberto Olivares Alarcos <aolivares@iri.upc.edu>, Institut de Robòtica i Informàtica Industrial, CSIC-UPC

import json

class JSONmodule:
    def __init__(self):
        pass
        
    def dictionary_to_json_string(self, dictionary):
        json_string = json.dumps(dictionary, indent=2) # indent is optional (just for clearer prints)

        return json_string

    def json_string_to_dictionary(self, json_string):
        dictionary = json.loads(json_string)

        return dictionary

