import setuptools
from setuptools import setup


setup(
    name='dynamic_obstacle_avoidance',
    version='1.0',
    description='Dynamic Obstacle Avoidance',
    author='Lukas Huber',
    author_email='lukas.huber@epfl.ch',
    packages=setuptools.find_packages(where="src", exclude=("tests",)),
    scripts=['scripts/examples_animation.py', 'scripts/examples_vector_field.py'],
    package_dir={'': 'src'}
    )
