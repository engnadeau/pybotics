import os
import glob
import subprocess
import logging

logging.basicConfig(level=logging.INFO)

logging.info('Testing Jupyter notebook examples')

example_path_pattern = os.path.join(os.path.dirname(__file__), '*.ipynb')
logging.info('Example path pattern:\t{}'.format(example_path_pattern))

example_paths = glob.glob(example_path_pattern)
logging.info('Found examples:\t{}'.format(example_paths))

for path in example_paths:
    logging.info('Testing:\t{}'.format(path))
    runnable = 'jupyter nbconvert --to script --execute --stdout {} | python'.format(path)
    subprocess.run(runnable)

logging.info('Example testing complete')
