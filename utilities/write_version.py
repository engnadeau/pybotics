from pathlib import Path
from setuptools_scm import get_version
import logging


def main():
    # get version
    version = get_version(root='..', relative_to=__file__)
    logging.info('Version: {}'.format(version))

    # ensure PEP440 compliance
    version_segments = version.split('.dev')
    if len(version_segments) > 1:
        logging.info('Cleaning version for PEP440 compliance')
        tag_version_segment = version_segments[0]
        dev_version_segment = version.split('.dev')[1]
        dev_version_segment = abs(hash(dev_version_segment))

        version = '{}.dev{}'.format(tag_version_segment, dev_version_segment)
        logging.info('PEP440 version: {}'.format(version))

    # get root path
    root_path = Path(__file__).parents[1]
    logging.info('Root path: {}'.format(root_path.resolve()))

    # dump version
    version_path = root_path / 'VERSION'
    logging.info('Dumping version to: {}'.format(version_path.resolve()))

    with open(str(version_path), 'w') as f:
        f.write(version)
    logging.info('Version dumped')


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    main()
