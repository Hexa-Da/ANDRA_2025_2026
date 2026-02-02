from setuptools import find_packages, setup

package_name = 'navigation_utils'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pillow', 'matplotlib'],
    zip_safe=True,
    maintainer='techlab',
    maintainer_email='techlab@robot-andra',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'odom_to_path = navigation_utils.odom_to_path:main',
            'report_fissures = navigation_utils.report_fissures:main',
            'show_pos = navigation_utils.show_pos:main',
            'test = navigation_utils.test:main',
            'sequence_robot = navigation_utils.sequence_robot:main',
        ],
    },
)
