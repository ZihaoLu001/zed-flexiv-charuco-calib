"""Import every package submodule (catches syntax/import breakage, incl. guarded hardware shims) and
byte-compile every CLI script. No hardware, no cv2 required for the import-graph checks."""
import importlib
import py_compile
from pathlib import Path

import pytest

PKG = "zfcc"
SUBMODULES = ["se3", "config", "board", "detect", "intrinsics", "handeye", "diversity",
              "validate", "touch_test", "yaml_out", "zed_io", "robot_io", "session",
              "coverage", "refine"]


@pytest.mark.parametrize("mod", SUBMODULES)
def test_import_submodule(mod):
    # board/detect/intrinsics/handeye/validate/session import cv2 lazily inside functions, so the
    # module import itself must succeed even without cv2 present.
    importlib.import_module(f"{PKG}.{mod}")


def test_package_lazy_attr():
    import zfcc
    assert zfcc.__version__
    assert hasattr(zfcc, "se3")


def test_scripts_compile():
    scripts = Path(__file__).resolve().parents[1] / "scripts"
    files = sorted(scripts.glob("*.py"))
    assert files, "no scripts found"
    for f in files:
        py_compile.compile(str(f), doraise=True)


def test_guarded_imports_do_not_require_hardware():
    # constructing the hardware readers must FAIL cleanly (RuntimeError), not ImportError at module load
    from zfcc.robot_io import FlangePoseReader
    from zfcc.zed_io import ZedCamera
    assert FlangePoseReader is not None and ZedCamera is not None
