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

from schema.scenario import SIMPLE_OPENSCENARIO_SCHEMA  # noqa: E402


def validate_scenario_file(scenario_file_path):
    """
    Validate a single scenario file against the schema.

    Args:
        scenario_file_path: Path object pointing to the scenario JSON file

    Returns:
        tuple: (bool, str) - (is_valid, error_message)
    """
    try:
        with open(scenario_file_path, "r") as file:
            scenario_data = json.load(file)

        validate(instance=scenario_data, schema=SIMPLE_OPENSCENARIO_SCHEMA)
        return True, None
    except json.JSONDecodeError as error:
        return False, f"JSON parsing error: {error}"
    except ValidationError as error:
        return False, f"Schema validation error: {error.message}"
    except Exception as error:
        return False, f"Unexpected error: {error}"


def validate_all_scenarios(scenario_directory):
    """
    Validate all JSON scenario files in the specified directory.

    Args:
        scenario_directory: Path to the directory containing scenario files

    Returns:
        int: Number of validation failures
    """
    scenario_dir_path = Path(scenario_directory)

    if not scenario_dir_path.exists():
        print(f"Error: Directory '{scenario_directory}' does not exist")
        return 1

    if not scenario_dir_path.is_dir():
        print(f"Error: '{scenario_directory}' is not a directory")
        return 1

    scenario_files = sorted(scenario_dir_path.glob("*.json"))

    if not scenario_files:
        print(f"No JSON files found in '{scenario_directory}'")
        return 0

    print(f"Validating {len(scenario_files)} scenario file(s)...\n")

    failures = 0
    for scenario_file in scenario_files:
        is_valid, error_message = validate_scenario_file(scenario_file)

        if is_valid:
            print(f"✓ {scenario_file.name}: VALID")
        else:
            print(f"✗ {scenario_file.name}: INVALID")
            print(f"  {error_message}\n")
            failures += 1

    print(f"\n{'=' * 60}")
    print(f"Total: {len(scenario_files)} file(s)")
    print(f"Valid: {len(scenario_files) - failures}")
    print(f"Invalid: {failures}")
    print(f"{'=' * 60}")

    return failures


def main():
    """Main entry point for the validation script."""
    script_dir = Path(__file__).parent.parent
    scenario_directory = script_dir / "data" / "scenario"

    failures = validate_all_scenarios(scenario_directory)

    if failures > 0:
        sys.exit(1)
    else:
        sys.exit(0)


if __name__ == "__main__":
    main()

# vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4 tw=79:
