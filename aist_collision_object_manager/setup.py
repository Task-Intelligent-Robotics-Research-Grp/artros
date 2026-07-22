from setuptools import setup, find_packages
from glob import glob

package_name = "aist_collision_object_manager"

setup(
    name=package_name,
    version="1.0.0",
    packages=find_packages(exclude=['test']),
    data_files=[
        ("share/ament_index/resource_index/packages",
         ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="Toshio Ueshiba",
    maintainer_email="t.ueshiba@aist.go.jp",
    description="Python interface for managing collision objects in MoveIt",
    license="BSD",
    tests_require=["pytest"],
)
