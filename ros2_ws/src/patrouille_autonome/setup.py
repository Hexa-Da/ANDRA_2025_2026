from setuptools import find_packages, setup

package_name = 'patrouille_autonome'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='techlab',
    maintainer_email='antoine.richard@depinfonancy.net',
    description='Orchestration de patrouille autonome (navigation + sequence photo/video).',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fusion_photo_navigation = patrouille_autonome.fusion_photo_navigation:main',
            'fusion_video_navigation = patrouille_autonome.fusion_video_navigation:main',
        ],
    },
)
