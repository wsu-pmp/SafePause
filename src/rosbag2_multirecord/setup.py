import os
import urllib.request
from glob import glob

from setuptools import find_packages, setup
from setuptools.command.build_py import build_py

package_name = "rosbag2_multirecord"


class DownloadMCAPBinary(build_py):
    # no apt package for mcap-cli
    # pymcap Python package downloads binaries at runtime - fails inside ROS b/c of directory perms
    # instead, download the binaries here at build time and manually retrieve them at runtime
    def run(self):
        bin_dir = os.path.join(self.build_lib, package_name, "bin")
        os.makedirs(bin_dir, exist_ok=True)

        mcap_url = "https://github.com/foxglove/mcap/releases/download/releases%2Fmcap-cli%2Fv0.0.51/mcap-linux-amd64"
        mcap_path = os.path.join(bin_dir, "mcap")

        urllib.request.urlretrieve(mcap_url, mcap_path)
        os.chmod(mcap_path, 0o755)

        super().run()


setup(
    name=package_name,
    version="0.0.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
        ("share/" + package_name, ["package.xml"]),
        ("share/" + package_name + "/launch", glob("launch/*.py")),
        ("share/" + package_name + "/config", glob("config/*.yaml")),
    ],
    install_requires=["setuptools"],
    zip_safe=True,
    maintainer="dev",
    maintainer_email="dev@todo.todo",
    description="TODO: Package description",
    license="TODO: License declaration",
    tests_require=["pytest"],
    cmdclass={
        "build_py": DownloadMCAPBinary,
    },
    package_data={
        package_name: ["bin/*"],
    },
    entry_points={
        "console_scripts": [
            "recorder = rosbag2_multirecord.rosbag2_recorder_node:main",
            "coordinator = rosbag2_multirecord.coordinator_node:main",
        ],
    },
)
