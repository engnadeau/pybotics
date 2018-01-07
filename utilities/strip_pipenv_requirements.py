import logging
from pathlib import Path


def main():
    # paths
    root_path = Path(__file__).parents[1]
    logging.info('Root path: {}'.format(root_path))

    # requirements
    requirement_paths = root_path.glob('*requirements.txt')

    for p in requirement_paths:
        logging.info('Cleaning: {}'.format(p))

        with open(str(p)) as f:
            requirements = f.read().splitlines()

        for i, req in enumerate(requirements):
            requirements[i] = req.split()[0]

        logging.info('Requirements: {}'.format(requirements))
        with open(str(p), 'w') as f:
            f.write('\n'.join(requirements))


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    main()
