#!/usr/bin/env python
import pkg_resources
import platform
import sys
import json

d = {
    'platform': platform.platform(),
    'machine': platform.machine(),
    'python': sys.version,
    'packages': []
}

pybotics_dist = pkg_resources.get_distribution('pybotics')
d['packages'].append({
    'name': pybotics_dist.key,
    'version': pybotics_dist.version
})

for dist in pkg_resources.get_distribution('pybotics').requires():
    d['packages'].append({
        'name': dist.key,
        'version': pkg_resources.get_distribution(dist.key).version
    })

print(json.dumps(d, sort_keys=True, indent=4))
