from setuptools import find_packages, setup

package_name = 'semantic_map_builder'

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
    maintainer='gs',
    maintainer_email='yuhanzhang0608@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'semantic_build = semantic_map_builder.semantic_map_manager:main',
            'test_semantic_build = semantic_map_builder.test_map_builder:main',
            'test_map_build = semantic_map_builder.test_map_build:main',
            'map_build = semantic_map_builder.semantic_map_builder:main'
        ],
    },
)
