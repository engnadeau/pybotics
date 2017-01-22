import git
import os

# tag version
file_path = os.path.abspath(os.path.dirname(__file__))
repo = git.Repo(file_path)
tag = repo.tags[-1]
version = tag.name

with open('VERSION', 'w') as f:
    f.write(version)
