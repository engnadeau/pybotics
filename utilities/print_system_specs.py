from platform import platform, machine

import pkg_resources
import json


def fetch_dependencies(package_name: str) -> dict:
    dist = pkg_resources.get_distribution(package_name)

    d = {'package': dist.key, 'version': dist.version}

    dependencies = dist.requires()
    dep_names = [d.key for d in dependencies]

    d['dependencies'] = list(map(fetch_dependencies, dep_names))

    return d


if __name__ == '__main__':
    d = fetch_dependencies('pybotics')
    d['platform'] = platform()
    d['machine'] = machine()

    print(json.dumps(d, indent=4))
