from setuptools import find_packages, setup

package_name = "emotion_active_perception"

setup(
    name=package_name,
    version="0.1.0",
    packages=find_packages("src"),
    package_dir={"": "src"},
    data_files=[
        ("share/ament_index/resource_index/packages", [f"resource/{package_name}"]),
        (f"share/{package_name}", ["package.xml"]),
        (f"share/{package_name}/config", [
            "src/emotion_active_perception/config/default.yaml",
            "src/emotion_active_perception/config/emotive_params.yaml",
        ]),
        (f"share/{package_name}/actions", [
            "src/emotion_active_perception/actions/MoveAndCapture.action",
        ]),
    ],
    install_requires=["setuptools", "pyyaml"],
    zip_safe=False,
    maintainer="maintainer",
    maintainer_email="devnull@example.com",
    description="Emotion-Guided Active Perception for PIC4rl_gym",
    license="MIT",
    entry_points={
        "console_scripts": [
            "ap_action_server = emotion_active_perception.nodes.ap_action_server:main",
            "emotion_controller = emotion_active_perception.nodes.emotion_controller_node:main",
            "uncertainty_node = emotion_active_perception.nodes.uncertainty_node:main",
            "auto_labeler_node = emotion_active_perception.nodes.auto_labeler_node:main",
            "data_manager_node = emotion_active_perception.nodes.data_manager_node:main",
            "example_drl_agent = emotion_active_perception.examples.example_drl_agent:main",
        ],
    },
)

