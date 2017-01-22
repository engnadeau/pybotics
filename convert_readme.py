import pypandoc
import os

print('Pandoc version: {}'.format(pypandoc.get_pandoc_version()))
print('Pandoc path: {}'.format(pypandoc.get_pandoc_path()))
print('Pandoc formats: {}'.format(pypandoc.get_pandoc_formats()))

file_path = os.path.abspath(os.path.dirname(__file__))
pypandoc.convert_file('README.md', 'rst', outputfile=os.path.join(file_path, 'README.rst'))
