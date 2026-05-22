from __future__ import annotations

import sys
import unittest
from pathlib import Path
from unittest.mock import patch


ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "src" / "go2_gui_controller"))

if "geometry_msgs.msg" in sys.modules:
    geometry_msgs_msg = sys.modules["geometry_msgs.msg"]
    for message_name in (
        "PointStamped",
        "PoseStamped",
        "PoseWithCovarianceStamped",
        "TransformStamped",
        "Vector3Stamped",
    ):
        if not hasattr(geometry_msgs_msg, message_name):
            setattr(geometry_msgs_msg, message_name, type(message_name, (), {}))

from go2_gui_controller import state_bridge as state_bridge_module
from go2_gui_controller.state_bridge import StateBridge


class _FakeNode:
    def create_subscription(self, *_args, **_kwargs):
        return object()


class StateBridgeTests(unittest.TestCase):
    def test_tf_buffer_is_bound_to_node_clock_for_sim_time_jumps(self):
        calls = []

        class FakeBuffer:
            def __init__(self, *args, **kwargs):
                calls.append((args, kwargs))

        node = _FakeNode()

        with (
            patch.object(state_bridge_module, "Buffer", FakeBuffer),
            patch.object(state_bridge_module, "TransformListener"),
        ):
            StateBridge(node)

        self.assertEqual(calls, [((), {"node": node})])


if __name__ == "__main__":
    unittest.main()
