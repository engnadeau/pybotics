from pybotics.robot_model import KukaLbrIiwa7, UR10, MecademicMeca500, Puma560


def test_models():
    # simply construct the models to ensure no errors
    KukaLbrIiwa7()
    UR10()
    MecademicMeca500()
    Puma560()
