from setuptools import find_packages, setup
from glob import glob

package_name = 'simulate_publish'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, glob("launch/py/*_launch.py")),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='zhq',
    maintainer_email='1325694319@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'left_img_publish = simulate_publish.left_img_publish:main',
            'right_img_publish = simulate_publish.right_img_publish:main',
            'pc_publish = simulate_publish.pc_publish:main',
            'odom_publish = simulate_publish.odom_publish:main',
        ],
    },
)
