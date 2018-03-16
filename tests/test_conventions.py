from collections import Counter

from pybotics.conventions import Orientation


def test_orientation():
    # ensure order and name match
    for e in list(Orientation.__members__.values()):
        name_order = e.name.split('_')[-1].lower()
        assert name_order == e.value

    # ensure that there are only two of each value (euler and fixed)
    values = [e.value for e in Orientation.__members__.values()]
    counts = Counter(values).values()
    assert all([v == 2 for v in counts])

    # ensure only x,y,z are used
    good_letters = set('xyz')
    values = set([e.value for e in Orientation.__members__.values()])
    leftover_values = [x for x in values if set(x).difference(good_letters)]
    assert not leftover_values
