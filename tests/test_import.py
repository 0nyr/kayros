import kayros


def test_version() -> None:
    assert kayros.__version__
    assert kayros._core.__version__ == kayros.__version__


def test_lera_ships_by_default() -> None:
    # M5.8: the exact BPC component (HiGHS backend) is part of the default
    # install (was gated behind KAYROS_WITH_LERA before 0.3.0).
    import importlib

    lera = importlib.import_module("kayros.lera")
    assert hasattr(lera, "solve_duration")
