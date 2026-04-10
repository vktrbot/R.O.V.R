from setuptools import find_packages, setup


package_name = "rovr_chassis"


setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages(),
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="rovr",
    maintainer_email="rovr@example.com",
    description="Chassis control node for ROVR.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "chassis_controller_node = rovr_chassis.node:main",
        ],
    },
)
