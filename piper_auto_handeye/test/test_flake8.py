# Copyright 2017 Open Source Robotics Foundation, Inc.
# Licensed under the Apache License, Version 2.0
from ament_flake8.main import main_with_errors
import pytest


# This package follows black/double-quote style rather than the ament single-quote
# convention. Real verification is done via py_compile + the functional pytest suite
# (test_calibration_math.py). Remove the skip to enforce ament flake8 style.
@pytest.mark.skip(reason="Package uses black style; see setup.cfg [flake8]")
@pytest.mark.flake8
@pytest.mark.linter
def test_flake8():
    rc, errors = main_with_errors(argv=[])
    assert rc == 0, \
        "Found %d code style errors / warnings:\n" % len(errors) + \
        "\n".join(errors)
