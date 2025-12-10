#!/usr/bin/env python3
# -*- coding: UTF-8 -*-
#
# Copyright (c) 2025, LaneZero Contributors
#
# All rights reserved.
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# - Redistributions of source code must retain the above copyright notice, this
#   list of conditions and the following disclaimer.
# - Redistributions in binary form must reproduce the above copyright notice,
#   this list of conditions and the following disclaimer in the documentation
#   and/or other materials provided with the distribution.
# - Neither the name of the copyright holder nor the names of its contributors
#   may be used to endorse or promote products derived from this software
#   without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

import json
import sys
from pathlib import Path

from jsonschema import validate, ValidationError

sys.path.insert(0, str(Path(__file__).parent.parent))

from schema.map import SIMPLE_OPENDRIVE_SCHEMA  # noqa: E402


def validate_map_file(map_file_path):
    """
    Validate a single map file against the schema.

    Args:
        map_file_path: Path object pointing to the map JSON file

    Returns:
        tuple: (bool, str) - (is_valid, error_message)
    """
    try:
        with open(map_file_path, 'r') as file:
            map_data = json.load(file)

        validate(instance=map_data, schema=SIMPLE_OPENDRIVE_SCHEMA)
        return True, None
    except json.JSONDecodeError as error:
        return False, f"JSON parsing error: {error}"
    except ValidationError as error:
        return False, f"Schema validation error: {error.message}"
    except Exception as error:
        return False, f"Unexpected error: {error}"


def validate_all_maps(map_directory):
    """
    Validate all JSON map files in the specified directory.

    Args:
        map_directory: Path to the directory containing map files

    Returns:
        int: Number of validation failures
    """
    map_dir_path = Path(map_directory)

    if not map_dir_path.exists():
        print(f"Error: Directory '{map_directory}' does not exist")
        return 1

    if not map_dir_path.is_dir():
        print(f"Error: '{map_directory}' is not a directory")
        return 1

    map_files = sorted(map_dir_path.glob('*.json'))

    if not map_files:
        print(f"No JSON files found in '{map_directory}'")
        return 0

    print(f"Validating {len(map_files)} map file(s)...\n")

    failures = 0
    for map_file in map_files:
        is_valid, error_message = validate_map_file(map_file)

        if is_valid:
            print(f"✓ {map_file.name}: VALID")
        else:
            print(f"✗ {map_file.name}: INVALID")
            print(f"  {error_message}\n")
            failures += 1

    print(f"\n{'='*60}")
    print(f"Total: {len(map_files)} file(s)")
    print(f"Valid: {len(map_files) - failures}")
    print(f"Invalid: {failures}")
    print(f"{'='*60}")

    return failures


def main():
    """Main entry point for the validation script."""
    script_dir = Path(__file__).parent.parent
    map_directory = script_dir / "data" / "map"

    failures = validate_all_maps(map_directory)

    if failures > 0:
        sys.exit(1)
    else:
        sys.exit(0)


if __name__ == "__main__":
    main()
