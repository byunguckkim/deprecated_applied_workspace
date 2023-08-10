# Copyright (C) 2018 Applied Intuition, Inc. All rights reserved.
# This source code file is distributed under and subject to the LICENSE in license.txt

from builtins import str
import datetime
import os
import signal
import subprocess
import time

from rosgraph_msgs.msg import Clock
import roslaunch
import rospy

from simian.public import stack_interface


class StackInterfaceRos(stack_interface.StackInterface):
    """ROS-specific customer stack interface."""

    def __init__(self, proc_handler, startup_options):
        super().__init__(proc_handler, startup_options)
        self._clock_pub = None
        self._clock_msg = Clock()
        self._roslaunch = None
        self._recorder = None

    def rosbag_record_args(self):
        return ["--all"]

    def create_recorder(self):
        return RosBagRecordProc()

    def get_roslaunch_file(self):
        raise NotImplementedError("Subclasses must implement get_roslaunch_file()")

    def _catch_signal(self, signum, _):
        if signum is signal.SIGINT:
            raise KeyboardInterrupt
        else:
            raise SystemExit

    def setup_middleware(self):
        assert self._roslaunch is None, "Must teardown middleware before setting up again"
        uuid = roslaunch.rlutil.get_or_generate_uuid(None, False)
        roslaunch.configure_logging(uuid)
        self._roslaunch = roslaunch.parent.ROSLaunchParent(
            uuid,
            [self.get_roslaunch_file()],
        )
        self._roslaunch.start(auto_terminate=False)
        rospy.init_node("sim_ros_bridge", disable_signals=True)
        # Must catch these signals after roslaunch is setup since it overrides
        # them no matter what.
        signal.signal(signal.SIGINT, self._catch_signal)
        signal.signal(signal.SIGTERM, self._catch_signal)

    def teardown_middleware(self):
        if self._roslaunch is not None:
            self._roslaunch.shutdown()
            self._roslaunch = None

    def start_recording(self, path):
        super().start_recording(path)
        assert self._recorder is None, "Must stop recording before starting again"
        self._recorder = self.create_recorder()
        if path.endswith(".bag"):
            self._recorder.start(self.rosbag_record_args(), path)
        elif not os.path.exists(path):
            os.makedirs(path)
            self._recorder.start(self.rosbag_record_args(), os.path.join(path, "output.bag"))
        else:
            raise Exception("Cannot write bag to path that already exists and is not a directory")

    def stop_recording(self):
        super().stop_recording()
        self._recorder.stop()  # type: ignore[union-attr] # ignore baseline; see #61427
        self._recorder = None

    def publish_time(self, timestamp):
        if self._clock_pub is None:
            # Late-initialize this publisher to avoid having to hit
            # the retry logic, which in some ROS implementations uses
            # the clock, causing this to get stuck not publishing the time.
            self._clock_pub = rospy.Publisher("/clock", Clock, queue_size=10)
        # timestamp is a datetime.datetime() from UTC time 0
        timestamp_seconds = (timestamp - datetime.datetime.utcfromtimestamp(0)).total_seconds()
        self._clock_msg.clock = rospy.Time(int(timestamp_seconds), timestamp.microsecond * 1000)
        self._clock_pub.publish(self._clock_msg)
        super().publish_time(timestamp)

    @property
    def recorded_data_path(self):
        return self._recorder.filename  # type: ignore[union-attr] # ignore baseline; see #61427


class RosBagRecordProc:
    """Handles a rosbag record process."""

    def __init__(self):
        self._proc = None

    def start(self, args, path):
        # Document which processes are currently running. We may later need
        # this to kill previously-running rosbag record calls.
        subprocess.Popen(["ps", "aux"]).wait()
        command = ["rosbag", "record", "-O", path] + args
        self._proc = subprocess.Popen(["bash", "-c", " ".join(command)], stdout=open("/dev/null"))

    def stop(self):
        # TODO(fahhem): Figure out a better way to kill rosbag-record's
        # 'record' child.
        # For some reason 'kill -INT $(pgrep -P %d)' causes sh to see
        # 'kill -INT ' instead of see a process.
        proc = subprocess.Popen(["pgrep", "-P", str(self._proc.pid)], stdout=subprocess.PIPE)  # type: ignore[union-attr] # ignore baseline; see #61427
        children_procs, _ = proc.communicate()

        if not children_procs.strip():
            # rosbag record didn't start properly.
            return

        while True:
            still_up = subprocess.Popen(["bash", "-c", "kill -0 {}".format(self._proc.pid)]).wait()  # type: ignore[union-attr] # ignore baseline; see #61427
            with open("/dev/null", "w") as devnull:
                if still_up == 0:
                    self._proc.poll()  # type: ignore[union-attr] # ignore baseline; see #61427
                    killer = subprocess.Popen(
                        ["bash", "-c", "kill -INT {}".format(children_procs)],  # type: ignore[str-bytes-safe] # ignore baseline; see #61427
                        stdout=devnull,
                        stderr=devnull,
                    )
                    killer.wait()
                    time.sleep(0.1)
                else:
                    break
        self._proc.wait()  # type: ignore[union-attr] # ignore baseline; see #61427
