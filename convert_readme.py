import pypandoc
import os

# TODO: pypandoc.convert() function returns wrong RST format, but saving/loading a file works
file_path = os.path.abspath(os.path.dirname(__file__))
pypandoc.convert_file('README.md', 'rst', outputfile=os.path.join(file_path, 'README'))
