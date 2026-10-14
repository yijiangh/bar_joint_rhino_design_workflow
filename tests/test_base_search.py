"""The base search's proposals around a picked base (`core.base_search.proposals`)."""

import numpy as np

GROUND = (np.array([[-5000.0, -5000.0, 0.0], [5000.0, -5000.0, 0.0], [5000.0, 5000.0, 0.0], [-5000.0, 5000.0, 0.0]]),
          np.array([[0, 1, 2], [0, 2, 3]]))
# Two arms, shoulders 0.26 m either side at 0.6 m height; flange targets 0.35 m either side of the origin at 1 m.
LIMITS = [(np.array([0.12, side * 0.26, 0.6]), np.array([dx, side * 0.35, 1.0]), 0.95)
          for side in (1, -1) for dx in (-0.05, 0.0)]
SETTINGS = dict(step_mm=50.0, yaw_steps=72, prefer=0.65, near_m=0.3, weight=0.25, spacing_mm=200.0, max_checks=300,
                refine_evals=40, radius_mm=1000.0, facing_range=np.pi / 2.0)


def _snap(point_mm):
    return np.array([point_mm[0], point_mm[1], 0.0]), np.array([0.0, 0.0, 1.0])


def _seed(x_mm, y_mm):
    frame = np.eye(4)
    frame[:3, 3] = (x_mm, y_mm, 0.0)
    return frame


def test_the_first_proposal_is_the_descent_from_the_seed():
    from core import base_search

    seed = _seed(-750.0, 100.0)
    first = next(base_search.proposals([GROUND], LIMITS, lambda frame: 1.0, _snap, seed_frame_mm=seed,
                                       log=lambda line: None, **SETTINGS))
    assert 1.0 < np.hypot(*(first[:2, 3] - seed[:2, 3])) < 400.0     # moved, but locally
    assert first[:3, 0] @ seed[:3, 0] > 0.0                           # still facing the seed's way


def test_the_search_carries_on_past_the_seed():
    from core import base_search

    search = base_search.proposals([GROUND], LIMITS, lambda frame: 1.0, _snap, seed_frame_mm=_seed(-750.0, 100.0),
                                   log=lambda line: None, **SETTINGS)
    assert len([frame for _, frame in zip(range(3), search)]) == 3


def test_nothing_is_proposed_where_the_robot_always_collides():
    from core import base_search

    found = list(base_search.proposals([GROUND], LIMITS, lambda frame: None, _snap, seed_frame_mm=_seed(-750.0, 100.0),
                                       log=lambda line: None, **SETTINGS))
    assert found == []
