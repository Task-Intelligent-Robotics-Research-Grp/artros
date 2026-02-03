#!/usr/bin/env python3

import yaml

def create_actions(name, entries):
    if 'type' in entries:
        return 'node-' + name

    actions = [ create_actions(n, e) for n, e in entries.items() ]
    if name == '':
        return str(actions)
    else:
        return  'group' + str(['ns-' + name] + actions)

with open('aist_new.yaml', 'r') as f:
    config = yaml.safe_load(f)

actions = create_actions('', config)

print(actions)
