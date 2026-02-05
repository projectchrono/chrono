"""CTest entry-point for PyChrono pytest suite.

Exit code 77 is used to indicate a skipped test (missing prerequisites).
"""

from __future__ import annotations

import argparse
import os
import sys


SKIP_RETURN_CODE = 77


def _skip(msg: str) -> int:
    sys.stderr.write(msg.rstrip() + "\n")
    return SKIP_RETURN_CODE


def main(argv: list[str]) -> int:
    parser = argparse.ArgumentParser(add_help=True)
    parser.add_argument("-q", "--quiet", action="store_true", help="Less verbose pytest output")
    parser.add_argument("path", nargs="?", default=os.getcwd(), help="Test directory (default: cwd)")
    args, extra = parser.parse_known_args(argv)

    try:
        import pytest  # noqa: F401
    except Exception as exc:  # pragma: no cover
        return _skip(f"pytest not available ({exc}). Install pytest to run PyChrono python tests.")

    try:
        import pychrono  # noqa: F401
    except Exception as exc:  # pragma: no cover
        py_path = os.environ.get("PYTHONPATH", "")
        return _skip(
            "pychrono not importable ({}).\n"
            "PYTHONPATH='{}'\n"
            "Enable/build CH_ENABLE_MODULE_PYTHON and ensure PYTHONPATH points to the build-tree modules."
            .format(exc, py_path)
        )

    pytest_args: list[str] = []
    if args.quiet:
        pytest_args.append("-q")
    pytest_args.append(args.path)
    pytest_args.extend(extra)

    import pytest

    return int(pytest.main(pytest_args))


if __name__ == "__main__":
    raise SystemExit(main(sys.argv[1:]))
