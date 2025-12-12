#!/bin/bash
# Convenience script to run examples with the correct environment

# Run the two-road example with visualization:
# ```bash
# ./run_example.sh examples/two_road_example.py --viewer
# ```
# Or without visualization:
# ```bash
# ./run_example.sh examples/two_road_example.py
# ```

SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
export PYTHONPATH="$SCRIPT_DIR:$PYTHONPATH"

exec python3.14 "$@"
