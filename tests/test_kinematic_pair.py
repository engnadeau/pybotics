from pybotics.kinematic_pair import KinematicPair


def test_unique():
    values = [e.value for e in KinematicPair]
    seen = set()
    assert not any(i in seen or seen.add(i) for i in values)
