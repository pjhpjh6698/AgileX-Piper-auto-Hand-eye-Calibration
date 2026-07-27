# Copyright 2015 Open Source Robotics Foundation, Inc.
# Licensed under the Apache License, Version 2.0
from ament_copyright.main import main
import pytest


# Remove the `skip` decorator once the source file(s) have a copyright header
@pytest.mark.skip(reason="No copyright header enforced in this package")
@pytest.mark.linter
def test_copyright():
    rc = main(argv=[".", "test"])
    assert rc == 0, "Found errors"
