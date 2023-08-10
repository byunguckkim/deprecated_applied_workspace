# Copyright (C) 2019 Applied Intuition, Inc. All rights reserved.
# This source code file is distributed under and subject to the LICENSE in
# license.txt

CLOUD_SIM_LOGDIR = "/var/log/simian"
CLOUD_SIM_RUNDIR = "/var/log/simian/running"
CLOUD_SIM_FINISHEDDIR = "/var/log/simian/finished"
# When running simulations in the cloud, the ADP_CLOUD_RUN environment
# variable will be set to "true" for all customer containers.
CLOUD_RUN_FLAG_ENV_VAR_NAME = "ADP_CLOUD_RUN"
CLOUD_RUN_NAMESPACE_ENV_VAR_NAME = "ADP_NAMESPACE"
CLOUD_DEFAULT_SPECTRAL_TIMEOUT_SECONDS = "60"
CUSTOMER_SERVER_VERSION = "1.0"
DEFAULT_ACTOR_SENSOR_NAME = "simian_perceived_actors"
SIMULATOR_START_TIME = 1e-6
DEFAULT_INTEGRATION_OPTIONS = [
    ("actor_reporting_version", {"major": 0, "minor": 1}),
    ("float_sim_time_compatibility", "FLOAT_SIM_TIME_LATEST"),
    ("multirate_stats_version", {"major": 0, "minor": 1}),
    ("actor_output_order", "SCENARIO_ORDER"),
    ("spectral_output_reporting", "SPECTRAL_OUTPUT_REPORTING_ALL_FIELDS"),
    ("time_synchronization_mode", "BETWEEN_CONVERT_TO_SIMIAN_CALLS"),
    ("ego_state_update_mode", "UPDATE_AT_ALL_CHANNEL_UPDATES"),
    ("transform_forest_actor_node_mode", "ACTOR_TF_NODES_ONLY_FOR_SPECTRAL_SENSORS"),
    ("actor_output_version", {"major": 0, "minor": 1}),
    ("yaml_defined_channel_timing_order_mode", "YAML_DEFINED_CHANNEL_ORDERING"),
    ("obstacle_3d_motion_state_consistency", {"major": 0, "minor": 3}),
    ("behavior_state_modifier", "FIRST_ORDER_BACKWARD_DIFFERENCE"),
    ("vertical_prism_dimension_version", {"major": 0, "minor": 2}),
    ("axis_aligned_box_dimension_version", {"major": 0, "minor": 2}),
    ("agent_state_output_mode", "FROM_MOTION_MODEL"),
]

ENABLE_DETAILED_RUNNER_LOGGING_FLAG: str = "enable_detailed_runner_logging"
HEALTH_CHECK_PORT: int = 10000
