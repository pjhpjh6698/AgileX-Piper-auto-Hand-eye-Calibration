# Copyright 2015 Open Source Robotics Foundation, Inc.
# Licensed under the Apache License, Version 2.0
from ament_pep257.main import main
import pytest


@pytest.mark.skip(reason="Package uses black style; see setup.cfg [flake8]")
@pytest.mark.linter
@pytest.mark.pep257
def test_pep257():
    rc = main(argv=[".", "test"])
    assert rc == 0, "Found code style errors / warnings"
