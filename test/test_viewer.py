#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

"""
Simple test to verify Qt GUI setup
"""

import pytest


def test_viewer_module_import():
    """Test that the viewer module can be imported."""
    from LaneZero.viewer import enable

    assert enable is not None


def test_viewer_enabled():
    """Test if Qt viewer is enabled."""
    from LaneZero.viewer import enable

    if not enable:
        pytest.skip("Qt GUI is not enabled.Rebuild with: cmake .. -DLANEZERO_USE_QT=ON")

    assert enable is True


def test_rmanager_import():
    """Test that RManager can be imported when Qt is enabled."""
    from LaneZero.viewer import enable

    if not enable:
        pytest.skip("Qt GUI is not enabled")

    from LaneZero.viewer import RManager

    assert RManager is not None


def test_rmanager_instance():
    """Test that RManager instance can be created when Qt is enabled."""
    from LaneZero.viewer import enable

    if not enable:
        pytest.skip("Qt GUI is not enabled")

    from LaneZero.viewer import RManager

    rm = RManager.get_instance()
    assert rm is not None


# vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4 tw=79:
