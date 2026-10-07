from setuptools import find_packages, setup

package_name = 'emma_manipulation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Bilal Gill',
    maintainer_email='5256190+bgill92@users.noreply.github.com',
    description='Arm motion planning for emma with roboplan.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'move_arm = emma_manipulation.move_arm:main',
        ],
    },
)
