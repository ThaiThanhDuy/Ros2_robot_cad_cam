from setuptools import setup

package_name = 'json_path_executor'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['json_path_executor/tcp_path.json']),
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
  
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
        'covert = json_path_executor.gcode_to_json_1:main',
        'visual = json_path_executor.display_planned_path:main',
        'line = json_path_executor.visual:main',
        'run = json_path_executor.run_path:main',
        'dao = json_path_executor.dao_simulation:main',
        'tcp = json_path_executor.tcp_marker_debugger:main',
        'phoi = json_path_executor.phoi_simulation:main',
        'san_pham = json_path_executor.san_pham_simulation:main',
        ],
    },
)
