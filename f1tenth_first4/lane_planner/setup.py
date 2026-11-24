from setuptools import setup

package_name = 'lane_planner'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='확인필요',
    maintainer_email='todo@example.com',
    description='카메라 차선 로컬 플래너 (LocalPlan 출력)',
    license='확인필요',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lane_planner_node = lane_planner.lane_planner_node:main',
            'calibrate_homography = lane_planner.calibrate_homography:main',
        ],
    },
)
