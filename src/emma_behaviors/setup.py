from glob import glob

from setuptools import find_packages, setup

package_name = 'emma_behaviors'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bilal Gill',
    maintainer_email='5256190+bgill92@users.noreply.github.com',
    description='Behaviour tree for a scripted pick and place with emma in sim.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'pick_and_place = emma_behaviors.pick_and_place:main',
        ],
    },
)
