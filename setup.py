from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'posha_robotic_sub'


def package_files(directory):
    files = []
    for path, _, filenames in os.walk(directory):
        if not filenames:
            continue
        files.append((os.path.join('share', package_name, path), [
                     os.path.join(path, name) for name in filenames]))
    return files


setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.py')),
    ] + package_files('config') + package_files('resource'),
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='rajeev',
    maintainer_email='rajeevkumawat2007@outlook.com',
    description='Posha autonomous robotic arm path planning.',
    license='MIT',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'generate_artifacts = posha_robotic_sub.generate_artifacts:main',
            'simulate_assignment = posha_robotic_sub.simulate_arm:main',
        ],
    },
)
