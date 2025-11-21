from setuptools import find_packages, setup

package_name = 'image_transfer'

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
    maintainer='techlab',
    maintainer_email='antoine.richard@depinfonancy.net',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_publisher = image_transfer.image_publisher:main',
            'image_subscriber = image_transfer.image_subscriber:main',
            'test = image_transfer.test:main',
            'show_pos = image_transfer.show_pos:main',
            'position_publisher = image_transfer.position_publisher:main',
            'report_fissures = image_transfer.report_fissures:main'
        ],
    },
)
