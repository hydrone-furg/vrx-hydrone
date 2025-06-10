import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'vrx_navigation'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Instala os arquivos de launch
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.launch.py'))),
        # Instala os arquivos de configuração
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dyfflen',
    maintainer_email='dyfflen@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    # Define os executáveis (seus nós Python)
    entry_points={
        'console_scripts': [
            'cmd = vrx_navigation.navigation_node_keyboard:main',
            'waypoint_navigator = vrx_navigation.waypoint_navigator:main',
            'timed = vrx_navigation.timed_thrust_node:main',
            # Adicione aqui outros nós se tiver, por exemplo:
            # 'meu_outro_no = vrx_navigation.meu_outro_no:main',
        ],
    },
)