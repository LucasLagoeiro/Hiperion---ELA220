from setuptools import find_packages, setup

package_name = 'finalExam_pkg'
submodules = "finalExam_pkg/submodules"

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name,submodules],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='wall-e',
    maintainer_email='llagoeiro@outlook.com.br',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'main_node = finalExam_pkg.main_node:main'
        ],
    },
)
