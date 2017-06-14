import git
import os
import logging

logging.basicConfig(level=logging.INFO)

logging.info('Updating package version info')

root_path = os.path.dirname(__file__)
logging.info('Root path:\t{}'.format(root_path))

repo = git.Repo(root_path)
tag = repo.tags[-1]
version = tag.name
logging.info('Latest git tag:\t{}'.format(version))

output_file = os.path.join(root_path, 'VERSION')
logging.info('Writing version info to:\t{}'.format(output_file))
with open(output_file, 'w') as f:
    f.write(version)

logging.info('Version update complete')
