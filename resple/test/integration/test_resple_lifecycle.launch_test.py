# Full-pipeline integration test (MANUAL / opt-in) — NOT run by CI.
#
# Brings up the real RESPLE lifecycle node and drives it through the
# configure -> activate -> deactivate -> cleanup transitions, asserting each
# succeeds and the process stays alive. This exercises the lifecycle / threading
# hardening (bounded joins, callback drains, publisher (de)activation) end to
# end. It needs a built ROS 2 (Jazzy) workspace with `resple` installed:
#
#     colcon build --packages-up-to resple
#     source install/setup.bash
#     launch_test src/RESPLE/resple/test/integration/test_resple_lifecycle.launch_test.py
#
# Optionally replay a bag in another terminal to push real PointCloud2/IMU data
# through the active node.

import time
import unittest

import launch
import launch_ros.actions
import launch_testing
import launch_testing.actions
import pytest
import rclpy
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition, State


@pytest.mark.launch_test
def generate_test_description():
    resple_node = launch_ros.actions.Node(
        package="resple",
        executable="RESPLE",
        name="resple",
        output="screen",
    )
    return (
        launch.LaunchDescription(
            [resple_node, launch_testing.actions.ReadyToTest()]
        ),
        {"resple_node": resple_node},
    )


class TestRespleLifecycle(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()
        cls.node = rclpy.create_node("resple_lifecycle_tester")

    @classmethod
    def tearDownClass(cls):
        cls.node.destroy_node()
        rclpy.shutdown()

    def _change_state(self, transition_id, timeout_sec=20.0):
        client = self.node.create_client(ChangeState, "/resple/change_state")
        self.assertTrue(
            client.wait_for_service(timeout_sec=timeout_sec),
            "lifecycle change_state service never appeared",
        )
        req = ChangeState.Request()
        req.transition = Transition(id=transition_id)
        future = client.call_async(req)
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
        self.assertIsNotNone(future.result(), "change_state call timed out")
        return future.result().success

    def _current_state(self, timeout_sec=10.0):
        client = self.node.create_client(GetState, "/resple/get_state")
        self.assertTrue(client.wait_for_service(timeout_sec=timeout_sec))
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self.node, future, timeout_sec=timeout_sec)
        return future.result().current_state.id

    def _await_state(self, wanted, timeout_sec=15.0):
        """Poll get_state until it reports `wanted`, then return."""
        deadline = time.time() + timeout_sec
        seen = None
        while time.time() < deadline:
            seen = self._current_state()
            if seen == wanted:
                return
            time.sleep(0.25)
        self.fail("node never reached state {} (last seen {})".format(wanted, seen))

    def test_full_lifecycle_recycle(self):
        # respleMain() drives configure() and activate() ITSELF before spinning
        # (see the RCLCPP_FATAL guards there), so the node is already ACTIVE
        # when this test starts. The previous version of this test opened with
        # TRANSITION_CONFIGURE, which is invalid from ACTIVE and always failed —
        # i.e. it could not pass, and so was never run.
        #
        # Start from ACTIVE and drive a full RE-CYCLE instead. That is also the
        # more valuable test: deactivate -> cleanup -> configure -> activate is
        # exactly the path hazards 17, 18, 71, 72, 76, 87 and the Mapping
        # zombie-worker guards all concern, and none of it is covered elsewhere.
        self._await_state(State.PRIMARY_STATE_ACTIVE)

        self.assertTrue(self._change_state(Transition.TRANSITION_DEACTIVATE),
                        "deactivate failed")
        self.assertEqual(self._current_state(), State.PRIMARY_STATE_INACTIVE)

        self.assertTrue(self._change_state(Transition.TRANSITION_CLEANUP),
                        "cleanup failed")
        self.assertEqual(self._current_state(),
                         State.PRIMARY_STATE_UNCONFIGURED)

        # The re-cycle proper. A declare_parameter without a has_parameter
        # guard (hazard 17) or an unreset resource (hazard 18/71/72/76) makes
        # THIS call fail or abort the process.
        self.assertTrue(self._change_state(Transition.TRANSITION_CONFIGURE),
                        "re-configure after cleanup failed")
        self.assertEqual(self._current_state(), State.PRIMARY_STATE_INACTIVE)

        self.assertTrue(self._change_state(Transition.TRANSITION_ACTIVATE),
                        "re-activate after cleanup failed")
        self.assertEqual(self._current_state(), State.PRIMARY_STATE_ACTIVE)

        # Let the freshly restarted worker thread spin.
        time.sleep(2.0)

        self.assertTrue(self._change_state(Transition.TRANSITION_DEACTIVATE),
                        "second deactivate failed")
        self.assertTrue(self._change_state(Transition.TRANSITION_CLEANUP),
                        "second cleanup failed")


@launch_testing.post_shutdown_test()
class TestRespleShutdown(unittest.TestCase):
    def test_clean_exit(self, proc_info, resple_node):
        # The bounded-join shutdown hardening should let the process exit
        # cleanly (no SIGSEGV/SIGABRT) after the lifecycle cycle.
        launch_testing.asserts.assertExitCodes(
            proc_info, allowable_exit_codes=[0, -2, 15], process=resple_node
        )
