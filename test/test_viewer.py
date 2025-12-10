#!/usr/bin/env python3
# -*- coding: UTF-8 -*-

"""
Simple test to verify Qt GUI setup
"""

import sys

# Test imports (without PySide6 environment setup)
print("Testing LaneZero Qt GUI implementation...")
print("=" * 60)

try:
    from LaneZero.viewer import enable
    print(f"✓ Qt viewer module imported successfully")
    print(f"✓ Qt viewer enabled: {enable}")

    if enable:
        from LaneZero.viewer import RManager, mgr
        print(f"✓ RManager class imported")
        print(f"✓ mgr singleton available")

        # Test RManager instance
        rm = RManager.get_instance()
        print(f"✓ RManager instance created: {rm}")

        print("\n" + "=" * 60)
        print("SUCCESS: All Qt GUI components are working!")
        print("=" * 60)
        print("\nTo launch the GUI:")
        print("  build/bin/LaneZeroView")
        sys.exit(0)
    else:
        print("\n" + "=" * 60)
        print("WARNING: Qt GUI is not enabled")
        print("Please rebuild with: cmake .. -DLANEZERO_USE_QT=ON")
        sys.exit(1)

except Exception as e:
    print(f"\n✗ Error: {e}")
    import traceback
    traceback.print_exc()
    sys.exit(1)

# vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4 tw=79:
