#!/usr/bin/env python3
"""Offline tests for public application metadata and version comparison."""

from __future__ import annotations

import os
import unittest

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

try:
    from a2c_sensor_tools.a2c_app_support import (
        APP_VERSION,
        github_latest_release_api_url,
        github_project_url,
        install_help_menu,
        is_newer_version,
    )
    from PySide6.QtWidgets import QApplication, QMainWindow
except ImportError as exc:  # pragma: no cover - depends on local Qt installation
    raise unittest.SkipTest(f"PySide6 unavailable: {exc}") from exc


class ApplicationSupportTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls) -> None:
        cls.application = QApplication.instance() or QApplication([])

    def test_semantic_version_comparison(self) -> None:
        self.assertTrue(is_newer_version("v1.2.4", "1.2.3"))
        self.assertTrue(is_newer_version("2.0.0", "v1.99.99"))
        self.assertFalse(is_newer_version("v1.2.3", "1.2.3"))
        self.assertFalse(is_newer_version("1.2.2", "1.2.3"))

    def test_stable_release_follows_prerelease(self) -> None:
        self.assertTrue(is_newer_version("1.2.3", "1.2.3-rc1"))
        self.assertFalse(is_newer_version("1.2.3-rc1", "1.2.3"))

    def test_invalid_release_tag_is_reported(self) -> None:
        self.assertIsNone(is_newer_version("firmware-0x0132", APP_VERSION))

    def test_github_urls_use_explicit_repository(self) -> None:
        repository = "example/a2c-sensor-tools"
        self.assertEqual(
            github_project_url(repository),
            "https://github.com/example/a2c-sensor-tools",
        )
        self.assertEqual(
            github_latest_release_api_url(repository),
            "https://api.github.com/repos/example/a2c-sensor-tools/releases/latest",
        )

    def test_empty_repository_disables_urls(self) -> None:
        self.assertEqual(github_project_url(""), "")
        self.assertEqual(github_latest_release_api_url(""), "")

    def test_help_menu_contains_update_and_about_actions(self) -> None:
        window = QMainWindow()
        checker = install_help_menu(window, "Test Application")
        self.assertIsNotNone(checker)
        help_menu = window._a2c_help_menu
        self.assertEqual(help_menu.title(), "&Help")
        labels = [action.text().replace("&", "") for action in help_menu.actions()]
        self.assertIn("Check for Updates…", labels)
        self.assertIn("About Test Application", labels)
        window.close()


if __name__ == "__main__":
    unittest.main()
