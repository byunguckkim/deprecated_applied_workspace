# Copyright (C) 2018 Applied Intuition, Inc. All rights reserved.
# This source code file is distributed under and subject to the LICENSE in license.txt


class StackInterface:
    """Interface to a customer's AV stack."""

    def __init__(self, proc_handler, startup_options):
        self._proc_handler = proc_handler
        self._options = startup_options

    def setup_middleware(self):
        """Optional function to setup the middleware."""

    def teardown_middleware(self):
        """Optional function to teardown the middleware."""

    def setup_stack(self):
        """Optional function to setup the customer stack."""

    def teardown_stack(self):
        """Optional function to teardown the customer stack."""

    def start_recording(self, path):
        """Optional function to begin recording using the middleware."""

    def stop_recording(self):
        """Optional function to stop recording using the middleware."""

    def start_visualization(self):
        """Optional function to start visualizing."""

    def stop_visualization(self):
        """Optional function to stop visualizing."""

    def publish_time(self, timestamp):
        """Accept the new timestamp into the stack/middleware."""

    def initialize(self, simulator_output, timestamp):
        """Initialize given the initial simulator output."""

    def get_simulation_hz(self):
        """Get the expected update rate of the simulation."""
        raise NotImplementedError("Choose a simulation rate, such as 10Hz or 40Hz.")

    def get_clock_hz(self):
        """Get the expected update rate of the simulated clock."""
        raise NotImplementedError("Choose a clock rate, such as 40Hz or 100Hz.")

    def get_stack_version(self):
        """The stack's version, called before anything else."""
        return "UNKNOWN"

    def finalize(self):
        """Finalize any local state."""

    def get_final_observers(self):
        """Optional function to get final observer events at simulation end"""
        return []

    def convert_and_publish_sim_outputs(self, simulator_output, tick_duration):
        """Convert and publish sim outputs/stack inputs."""
        raise NotImplementedError("Must implement and handle simulator outputs / stack inputs.")

    def convert_and_return_listened_sim_inputs(self):
        """Return converted sim inputs/stack outputs that were last received."""
        raise NotImplementedError("Must implement and return simulator inputs / stack outputs.")
