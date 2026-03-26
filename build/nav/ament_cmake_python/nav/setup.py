from setuptools import find_packages
from setuptools import setup

setup(
    name='nav',
    version='0.0.0',
    packages=find_packages(
        include=('nav', 'nav.*')),
)
