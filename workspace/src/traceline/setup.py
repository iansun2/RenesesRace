from setuptools import find_packages, setup

package_name = 'traceline'

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
    maintainer='root',
    maintainer_email='root@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'traceline_r_node = traceline.tracelineR:main',
            'traceline_l_node = traceline.tracelineL:main',
            'traceline_new_node = traceline.tracelineNew:main'
        ],
    },
)
