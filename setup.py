from setuptools import find_packages, setup
from glob import glob

package_name = 'kirox_robot'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),  # ← 這行
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='del1215',
    maintainer_email='dennyluu1215@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robotbrain = kirox_robot.robotbrain:main',
            'robotface = kirox_robot.robotface:main',
            'roboteyes = kirox_robot.roboteyes:main',
            'robotears = kirox_robot.robotears:main',
            'wsconnection = kirox_robot.ws_connection:main',
            'testnode = kirox_robot.testnode:main',
            'camera = kirox_robot.kirox_camera:main',
        ],
    },
)
