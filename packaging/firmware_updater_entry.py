#!/usr/bin/env python3
# Copyright 2026 A2C
# SPDX-License-Identifier: Apache-2.0
"""Frozen application entry point for the graphical firmware updater."""

from a2c_sensor_tools.can_firmware_update_qt import main


if __name__ == "__main__":
    raise SystemExit(main())
