from setuptools import setup

package_name = 'hl7_bridges'

setup(
    name=package_name,
    version='0.0.0',
    packages=['hl7_bridges'],
    py_modules=[],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/dp1_bridges.launch.py']),
        ('share/' + package_name, ['launch/dp2']),
    ],
    install_requires=['setuptools'],
    author='Morgan Quigley',
    author_email='morgan@osrfoundation.org',
    zip_safe=False,
    maintainer='Morgan Quigley',
    maintainer_email='morgan@osrfoundation.org',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='hl7_bridges',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hl7_mllp_client=hl7_bridges.hl7_mllp_client:main',
            'hl7_mllp_server=hl7_bridges.hl7_mllp_server:main',
        ],
    },
)
