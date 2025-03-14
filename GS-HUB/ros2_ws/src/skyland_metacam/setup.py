from setuptools import find_packages, setup

package_name = 'skyland_metacam'

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
    maintainer_email='gs@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_record = skyland_metacam.metacam_subscriber:main',
            'semantic_service = skyland_metacam.metacam_service:main',
            'semantic_test = skyland_metacam.test_metacam_service:main',
        ],
    },
)
