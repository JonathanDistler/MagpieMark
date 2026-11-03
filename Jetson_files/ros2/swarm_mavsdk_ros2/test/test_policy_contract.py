import math
import time
from unittest.mock import Mock
import pytest
from drone_policy import Observation, PolicyController


def backend(age=0):
    b = Mock()
    b.observation.return_value = Observation((1,2,-3), (0,0,0), time.monotonic()-age)
    return b


def test_diagonal_speed_is_bounded():
    b = backend()
    PolicyController(b, max_speed_mps=2).apply([1,1,1])
    assert math.hypot(*b.send_velocity_ned.call_args.args[0]) == pytest.approx(2)


@pytest.mark.parametrize('action', [[float('nan'),0,0], [1,2], [float('inf'),0,0]])
def test_invalid_actions_stop_without_publishing(action):
    b = backend()
    with pytest.raises(ValueError):
        PolicyController(b).apply(action)
    b.stop.assert_called_once()
    b.send_velocity_ned.assert_not_called()


def test_stale_state_stops_without_publishing():
    b = backend(age=3)
    with pytest.raises(RuntimeError):
        PolicyController(b).apply([0,0,0])
    b.stop.assert_called_once()
    b.send_velocity_ned.assert_not_called()


def test_observation_order():
    assert Observation((1,2,3),(4,5,6),0).vector() == (1,2,3,4,5,6)


def test_inference_failure_stops():
    b = backend()
    policy = Mock()
    policy.act.side_effect = RuntimeError('inference failed')
    with pytest.raises(RuntimeError, match='inference'):
        PolicyController(b).tick(policy)
    b.stop.assert_called_once()
    b.send_velocity_ned.assert_not_called()
