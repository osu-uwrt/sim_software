#!/usr/bin/env python

import yaml

with open("thruster2com.yaml", 'r') as stream:
    try:
        x = yaml.load(stream)
    except yaml.YAMLError as exc:
        print(exc)

for k, v in x.items():
    lol = ""
    for j, n in v.items():
        lol = lol + " " + str(n)
    
    print(lol)
