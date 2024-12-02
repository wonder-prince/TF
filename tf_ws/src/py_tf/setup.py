from setuptools import find_packages, setup

package_name = 'py_tf'

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
    maintainer='interstellar',
    maintainer_email='20235459@stu.cqu.edu.cn',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        'sta_broadcaster = py_tf.sta_broadcaster:main',
        'dy_broadcaster = py_tf.dy_broadcaster:main',
        'listener = py_tf.listener:main'     
        ],
    },
)
