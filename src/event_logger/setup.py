from setuptools import find_packages, setup

package_name = "event_logger"

setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="sp",
    maintainer_email="sp@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    entry_points={},
)
