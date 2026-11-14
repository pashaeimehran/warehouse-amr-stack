from setuptools import find_packages, setup

package_name = "perception_core"

setup(
    name=package_name,
    version="0.3.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Mehran Pashaei",
    maintainer_email="mehran.pashaei.mp@gmail.com",
    description="ROS-independent perception core",
    license="Apache-2.0",
    tests_require=["pytest"],
)
