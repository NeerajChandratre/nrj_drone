from setuptools import find_packages, setup

package_name = 'px4_ros2_multidrone'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='neeraj',
    maintainer_email='neerajcc2012@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'px4_ros2_multidrone_run = px4_ros2_multidrone.px4_ros2_multidrone_run:main'
        ],
    },
)
