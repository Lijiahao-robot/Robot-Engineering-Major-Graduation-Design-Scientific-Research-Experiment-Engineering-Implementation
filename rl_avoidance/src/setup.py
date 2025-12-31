from setuptools import setup
import os
from glob import glob

package_name = 'rl_avoidance'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'models'), glob('models/*')),
    ],
    install_requires=['setuptools', 'gymnasium', 'stable-baselines3', 'numpy'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@example.com',
    description='RL Obstacle Avoidance',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'rl_avoid_node = rl_avoidance.rl_avoid_node:main'
        ],
    },
)
