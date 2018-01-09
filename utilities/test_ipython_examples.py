import logging
from pathlib import Path
import subprocess


def main():
    # path
    examples_path = Path(__file__).parents[1] / 'examples'  # type: Path
    logging.info('Examples path: {}'.format(examples_path))

    # glob examples
    examples = list(examples_path.glob('*.ipynb'))
    logging.info('Globbed {} examples'.format(len(examples)))

    # test
    for ex in examples:
        logging.info('Testing: {}'.format(ex.resolve()))
        subprocess.run(['jupyter', 'nbconvert',
                        '--to', 'script',
                        '--execute',
                        '--stdout',
                        '{}'.format(str(ex))])


if __name__ == '__main__':
    logging.basicConfig(level=logging.INFO)
    main()
