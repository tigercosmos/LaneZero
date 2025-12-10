#!/usr/bin/env bash
# Setup environment variables for LaneZero Qt viewer

# Find PySide6 Qt directory
PYSIDE_QT=$(python3 -c "import PySide6; import os; print(os.path.join(os.path.dirname(PySide6.__file__), 'Qt'))")

if [ -d "$PYSIDE_QT" ]; then
    export DYLD_LIBRARY_PATH="$PYSIDE_QT/lib:$DYLD_LIBRARY_PATH"
    export QT_PLUGIN_PATH="$PYSIDE_QT/plugins"
    export DYLD_FRAMEWORK_PATH="$PYSIDE_QT/lib"
    echo "PySide6 Qt environment configured:"
    echo "  PYSIDE_QT=$PYSIDE_QT"
    echo "  DYLD_LIBRARY_PATH=$DYLD_LIBRARY_PATH"
    echo "  QT_PLUGIN_PATH=$QT_PLUGIN_PATH"
    echo "  DYLD_FRAMEWORK_PATH=$DYLD_FRAMEWORK_PATH"
else
    echo "Warning: PySide6 Qt directory not found at $PYSIDE_QT"
fi

# vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4:
