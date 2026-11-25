import kayros


def test_version() -> None:
    assert kayros.__version__
    assert kayros._core.__version__ == kayros.__version__


def test_lera_ships_by_default() -> None:
    # M5.8: the exact BPC component (HiGHS backend) is part of the default
    # install (was gated behind KAYROS_WITH_LERA before 0.3.0).
    import importlib
    from typing import get_type_hints

    exact = importlib.import_module("kayros.exact")
    assert callable(exact.solve_exact)
    assert get_type_hints(kayros.Incumbent)["origin"] is str
    assert importlib.util.find_spec("kayros.lera") is None
