"""
Pytest configuration and fixture harness for rover-tasks-template.
Configures python path to include workspace root and provides reusable mock rover fixtures.
"""

import os
import sys
import pytest

# Add parent directory of tests (rover-tasks-template root) to sys.path
root_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if root_dir not in sys.path:
    sys.path.insert(0, root_dir)

from api.rover_client import RoverClient
from api.mock_rover import MockRover


@pytest.fixture
def mock_client():
    """Provides a fresh RoverClient instance running in mock simulation mode."""
    client = RoverClient(mock=True)
    yield client
    client.stop()


@pytest.fixture
def mock_rover():
    """Provides a standalone MockRover simulation instance."""
    return MockRover()
