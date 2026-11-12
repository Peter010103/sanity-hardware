from setuptools import find_packages, setup

setup(
    name="sanity_zenoh",
    version="0.1.0",
    packages=find_packages(exclude=["test"]),
    data_files=[
        ("share/sanity_zenoh/launch", ["launch/cable.launch.py", "launch/plate.launch.py"]),
        ("share/ament_index/resource_index/packages", ["resource/sanity_zenoh"]),
        ("share/sanity_zenoh", ["package.xml", "README.md", "LOCAL_POLICY.md", "ROS_SEQUENCE.md", "EXPERIMENT.md", "MPC_BASELINE.txt", "INDI_HOVER.txt", "FULL_INDI_FLIGHT.txt", "CABLE_DEPLOYMENT.md", "PLATE_DEPLOYMENT.md", "AUTOTUNE.txt", "COMMS_COMPARE.txt", "FLEET_FLIPS.md", "requirements.txt"]),
        ("share/sanity_zenoh/config", ["config/paper_native_replay_example.json", "config/sanity15_paper_ctbr_compatibility_1khz.json", "config/sanity15_full_indi_estimator_hover.json",
            "config/sanity15_full_indi_estimator_hover_diagnostics.json",
            "config/sanity15_full_indi_estimator_hover_1khz.json",
            "config/sanity15_indi_estimator_hover_1khz.json",
            "config/sanity15_pid_estimator_hover_1khz.json"]),
        ("share/sanity_zenoh/config", ["config/sanity15_indi_hover.json", "config/sanity15_indi_collective.json", "config/sanity09_hx711_payload_23g_20261028.json", "config/sanity09_hx711_hover_20261028.json", "config/cable_engagement_shadow.json", "config/sanity_plate_108g.json", "config/sanity_cable.json", "config/sanity_cable_room_20261024.json", "config/sanity_cable_room_23g_20261024.json", "config/sanity_payload_spiral_23g_20261024.json", "config/sanity_cable_faster_23g_20261024.json", "config/sanity_cable_faster_comparison_aware_23g_20261025.json", "config/sanity_cable_faster_unaware_23g_20261025.json", "config/sanity_cable_payload_23g.json", "config/zenoh.yaml", "config/sanity01_direct.yaml", "config/mocap.yaml", "config/dds_robotlab.xml", "config/circle_sequence.yaml", "config/pitch_loop_sequence.yaml", "config/sanity09_pitch_loop.json", "config/sanity09_mpc.json", "config/sanity09_mpc_bias.json", "config/fleet_hover_five.yaml", "config/fleet_hover_six.yaml", "config/fleet_hover_registry.yaml", "config/sanity14_esp_collective.json", "config/sanity18_esp_collective.json", "config/sanity10_esp_collective.json", "config/sanity22_esp_collective.json", "config/sanity09_mpc_5ms.yaml", "config/sanity09_mpc_high_bank.json", "config/sanity09_motor_collective.json", "config/sanity09_esp_collective.json"]),
    ],
    install_requires=["setuptools", "eclipse-zenoh==1.10.1", "numpy", "scipy", "matplotlib", "casadi==3.7.2"],
    maintainer="Sanity",
    maintainer_email="hw527@cam.ac.uk",
    description="Sanity Zenoh command transport",
    license="TODO",
    entry_points={
        "console_scripts": [
            "sanity-test-comms = sanity_zenoh.comms_test:main",
            "sanity-comms-compare = sanity_zenoh.comms_compare:main",
            "sanity-indi-hover = sanity_zenoh.indi_hover:main",
            "sanity-paper-native = sanity_zenoh.paper_native:main",
            "sanity-experiment = sanity_zenoh.experiment:main",
            "sanity_sequence = sanity_zenoh.ros_sequence:main",
            "sanity_cable = sanity_zenoh.cable_sequence:main",
            "sanity_cable_shadow = sanity_zenoh.cable_shadow_bench:main",
            "sanity_plate = sanity_zenoh.plate_sequence:main",
            "sanity_plate_hover = sanity_zenoh.plate_commissioning:main",
            "sanity-clocked-sequence-sim = sanity_zenoh.clocked_sequence_sim:main",
            "sanity-simulate-sequence = sanity_zenoh.simulate_sequence:main",
            "sanity_mocap = sanity_zenoh.mocap_bridge:main",
            "sanity_mocap_stream = sanity_zenoh.mocap_stream_node:main",
            "sanity-local-policy = sanity_zenoh.local_policy:main",
            "zenoh_sender = sanity_zenoh.ros_bridge:main",
            "sanity-zenoh-send = sanity_zenoh.cli:main",
        ]
    },
)
