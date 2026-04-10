from setuptools import find_packages, setup


package_name = "rovr_arm"


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
    description="Arm control node for ROVR.",
    license="MIT",
    entry_points={
        "console_scripts": [
            "arm_controller_node = rovr_arm.node:main",
        ],
    },
)
