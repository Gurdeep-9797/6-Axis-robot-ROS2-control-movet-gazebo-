from setuptools import setup

package_name = "roboforge_bridge"

setup(
    name=package_name,
    version="1.0.0",
    packages=[f"{package_name}"],
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Robot Team",
    maintainer_email="robot@example.com",
    description="RoboForge Bridge - WebSocket server with rosbridge protocol",
    license="MIT",
    tests_require=["pytest"],
    entry_points={
        "console_scripts": [
            "roboforge_bridge = roboforge_bridge.bridge_node:main",
            "encoder_interface_node = roboforge_bridge.encoder_interface_node:main",
            "motor_controller_node = roboforge_bridge.motor_controller_node:main",
            "gazebo_error_node = roboforge_bridge.gazebo_error_node:main",
        ],
    },
)
