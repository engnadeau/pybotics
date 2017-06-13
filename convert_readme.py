import pypandoc
import os
import logging

logging.basicConfig(level=logging.INFO)

logging.info('Converting markdown README to RST for PyPI')
logging.info('Pandoc version:\t{}'.format(pypandoc.get_pandoc_version()))

root_path = os.path.dirname(__file__)
logging.info('Root path:\t{}'.format(root_path))

input_file = os.path.join(root_path, 'README.md')
output_file = os.path.join(root_path, 'README.rst')

logging.info('Input file:\t{}'.format(input_file))
logging.info('Output file:\t{}'.format(output_file))

pypandoc.convert_file(input_file, 'rst', outputfile=os.path.join(root_path, output_file))
logging.info('Conversion complete')
