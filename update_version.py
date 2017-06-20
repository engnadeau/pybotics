import git
import os
import logging

logging.basicConfig(level=logging.INFO)

logging.info('Updating package version info')

root_path = os.path.dirname(__file__)
logging.info('Root path:\t{}'.format(root_path))

repo = git.Repo(root_path)
logging.info('Repo:\t{}'.format(repo))

latest_tag = repo.tags[-1]
logging.info('Latest tag:\t{}'.format(latest_tag))

travis_commit = os.environ.get('TRAVIS_COMMIT')
logging.info('Travis commit:\t{}'.format(travis_commit))

sha = repo.head.object.hexsha
logging.info('Last commit sha:\t{}'.format(sha))

short_sha = repo.git.rev_parse(sha, short=4)
logging.info('Last commit short sha:\t{}'.format(short_sha))

travis_branch = os.environ.get('TRAVIS_BRANCH')
logging.info('Travis branch:\t{}'.format(travis_branch))

travis_pr_branch = os.environ.get('TRAVIS_PULL_REQUEST_BRANCH')
logging.info('Travis PR branch:\t{}'.format(travis_pr_branch))

travis_tag = os.environ.get('TRAVIS_TAG')
logging.info('Travis tag:\t{}'.format(travis_tag))

if len(travis_tag) > 0:
    version = '{}'.format(latest_tag)
else:
    version = '{}+{}'.format(latest_tag, short_sha)

logging.info('Package version:\t{}'.format(version))

output_file = os.path.join(root_path, 'VERSION')
logging.info('Writing version info to:\t{}'.format(output_file))
with open(output_file, 'w') as f:
    f.write(version)

logging.info('Version update complete')
