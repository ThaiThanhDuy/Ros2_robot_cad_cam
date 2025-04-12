from setuptools import setup

package_name = 'json_path_executor'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['json_path_executor/tcp_path.json']),
    ],
    install_requires=['setuptools', 'transforms3d'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='you@example.com',
    description='Run TCP path from JSON using MoveIt2 and Panda arm',
    license='MIT',
    entry_points={
    'console_scripts': [
        'run_path = json_path_executor.json_to_moveit:main',
        'covert = json_path_executor.gcode_to_json:main',
        'visual = json_path_executor.display_planned_path:main',

        ],
    },
)
