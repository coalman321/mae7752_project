from setuptools import setup

package_name = 'avoidance_planner'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='coalman321',
    maintainer_email='cjtucker321@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'planner = avoidance_planner.planner:main',
        ],
    },
    py_modules=[
        "avoidance_planner.trajectory.functions",
        "avoidance_planner.trajectory.helpers",
        "avoidance_planner.trajectory.constants",
        "avoidance_planner.trajectory.trajectory"
    ]
)
