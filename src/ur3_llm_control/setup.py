from glob import glob
import os

from setuptools import setup

package_name = "ur3_llm_control"

data_files = [
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml", "README.md"]),
    (os.path.join("share", package_name, "launch"), glob("launch/*.launch.py")),
    (os.path.join("share", package_name, "config"), glob("config/*.yaml")),
    (os.path.join("share", package_name, "worlds"), glob("worlds/*.sdf")),
    (os.path.join("share", package_name, "urdf"), glob("urdf/*.xacro")),
]
for config_dir in glob("config/*"):
    if os.path.isdir(config_dir):
        data_files.append((
            os.path.join("share", package_name, config_dir.replace(os.sep, "/")),
            glob(os.path.join(config_dir, "*")),
        ))
for model_dir in glob("models/*"):
    if os.path.isdir(model_dir):
        data_files.append((
            os.path.join("share", package_name, model_dir.replace(os.sep, "/")),
            glob(os.path.join(model_dir, "*")),
        ))

setup(
    name=package_name,
    version="0.2.0",
    packages=[package_name],
    data_files=data_files,
    install_requires=["setuptools", "PyYAML"],
    zip_safe=True,
    maintainer="Lục Văn Khoa",
    maintainer_email="khoa@todo.todo",
    description="LLM skill planning for UR3/UR3e with Gazebo, MoveIt 2, a suction gripper and camera.",
    license="Apache-2.0",
    entry_points={
        "console_scripts": [
            "llm_planner = ur3_llm_control.llm_planner:main",
            "skill_executor = ur3_llm_control.skill_executor:main",
            "camera_perception = ur3_llm_control.camera_perception:main",
        ],
    },
)
