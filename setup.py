from setuptools import setup

package_name = "chess_arm_bridge"

setup(
    name=package_name,
    version="0.1.0",
    packages=[package_name],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/config", ["config/example.json"]),
    ],
    install_requires=["setuptools", "python-chess>=1.999"],
    zip_safe=True,
    entry_points={"console_scripts": [
        "chess_move_plan = chess_arm_bridge.cli:main",
        "chess_move_gz = chess_arm_bridge.gazebo_cli:main",
    ]},
)
