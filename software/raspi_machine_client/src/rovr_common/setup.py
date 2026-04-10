from setuptools import find_packages, setup


package_name = "rovr_common"


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
    description="Shared utilities and mock backends for ROVR.",
    license="MIT",
)
