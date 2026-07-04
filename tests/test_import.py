import kayros


def test_version() -> None:
    assert kayros.__version__
    assert kayros._core.__version__ == kayros.__version__


def test_lera_is_gated() -> None:
    import importlib

    import pytest

    with pytest.raises(ImportError, match="KAYROS_WITH_LERA"):
        importlib.import_module("kayros.lera")
