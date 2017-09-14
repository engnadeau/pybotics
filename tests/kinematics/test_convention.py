from pybotics.kinematics.convention import Convention


def test_unique():
    values = [e.value for e in Convention]
    seen = set()
    assert not any(i in seen or seen.add(i) for i in values)
