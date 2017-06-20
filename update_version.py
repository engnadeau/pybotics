import git
import os
import logging

logging.basicConfig(level=logging.INFO)

logging.info('Updating package version info')

root_path = os.path.dirname(__file__)
logging.info('Root path:\t{}'.format(root_path))

repo = git.Repo(root_path)
latest_tag = repo.tags[-1]
current_branch = repo.active_branch
sha = repo.head.object.hexsha
short_sha = repo.git.rev_parse(sha, short=4)

logging.info('Repo:\t{}'.format(repo))
logging.info('Latest tag:\t{}'.format(latest_tag))
logging.info('Current branch:\t{}'.format(current_branch))
logging.info('Last commit sha:\t{}'.format(sha))
logging.info('Last commit short sha:\t{}'.format(short_sha))

if str(current_branch) == 'master':
    version = '{}'.format(latest_tag)
else:
    version = '{}+{}'.format(latest_tag, short_sha)

logging.info('Package version:\t{}'.format(version))

output_file = os.path.join(root_path, 'VERSION')
logging.info('Writing version info to:\t{}'.format(output_file))
with open(output_file, 'w') as f:
    f.write(version)

logging.info('Version update complete')
