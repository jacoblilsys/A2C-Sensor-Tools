#!/usr/bin/env python3
# Copyright 2026 A2C
# SPDX-License-Identifier: Apache-2.0
"""Frozen application entry point for the firmware update helper."""

from a2c_sensor_tools.can_firmware_update import main


if __name__ == "__main__":
    raise SystemExit(main())
