"""Verify that slow status flags do not mask stale pose or estimated state."""
import importlib.util
from pathlib import Path

import pytest

path = Path(__file__).resolve().parents[3] / 'scripts/check_external_odometry.py'
spec = importlib.util.spec_from_file_location('odometry_checker', path)
checker = importlib.util.module_from_spec(spec)
spec.loader.exec_module(checker)


def observations():
    counts = {(0, 'input'): 50, (0, 'local'): 50, (0, 'flags'): 3}
    last = {(0, 'input'): (9.9, None), (0, 'local'): (9.9, None),
            (0, 'flags'): (7., None)}
    return counts, last


def test_slow_status_cadence_keeps_fresh_pose_and_state_valid():
    counts, last = observations()
    assert checker.samples_fresh(0, counts, last, 10., 5.)


@pytest.mark.parametrize('kind', ['input', 'local', 'flags'])
def test_stale_or_missing_stream_fails(kind):
    counts, last = observations()
    last[0, kind] = (0., None)
    assert not checker.samples_fresh(0, counts, last, 10., 5.)
    del last[0, kind]
    assert not checker.samples_fresh(0, counts, last, 10., 5.)


def test_other_vehicle_never_satisfies_freshness():
    counts, last = observations()
    assert not checker.samples_fresh(1, counts, last, 10., 5.)
