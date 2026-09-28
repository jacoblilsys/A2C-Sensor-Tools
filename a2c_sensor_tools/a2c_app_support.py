# Copyright 2026 A2C
# SPDX-License-Identifier: Apache-2.0
"""Shared application metadata, About menu, and release update checking."""

from __future__ import annotations

import json
import os
import re
from typing import Optional

from PySide6.QtCore import QObject, QUrl
from PySide6.QtGui import QAction, QDesktopServices
from PySide6.QtNetwork import QNetworkAccessManager, QNetworkReply, QNetworkRequest
from PySide6.QtWidgets import QMainWindow, QMessageBox

from .version import APP_VERSION


APP_SUITE_NAME = "A2C Sensor Tools"

# Set this to "owner/repository" when the public GitHub repository is created.
# The environment override is useful for testing a build before publication.
DEFAULT_GITHUB_REPOSITORY = "jacoblilsys/A2C-Sensor-Tools"
GITHUB_REPOSITORY = os.environ.get(
    "A2C_SENSOR_TOOLS_GITHUB_REPOSITORY", DEFAULT_GITHUB_REPOSITORY
).strip()

APACHE_LICENSE_URL = "https://www.apache.org/licenses/LICENSE-2.0"
KVASER_SDK_URL = "https://kvaser.com/canlib-sdk/"

_VERSION_PATTERN = re.compile(
    r"^[vV]?(\d+)\.(\d+)\.(\d+)(?:[-.]([0-9A-Za-z.-]+))?(?:\+[0-9A-Za-z.-]+)?$"
)


def github_project_url(repository: str = GITHUB_REPOSITORY) -> str:
    return f"https://github.com/{repository}" if repository else ""


def github_latest_release_api_url(repository: str = GITHUB_REPOSITORY) -> str:
    return f"https://api.github.com/repos/{repository}/releases/latest" if repository else ""


def _version_key(version: str) -> Optional[tuple[int, int, int, int, str]]:
    """Return a comparison key for the SemVer forms used by app releases."""
    match = _VERSION_PATTERN.fullmatch(version.strip())
    if match is None:
        return None
    major, minor, patch = (int(match.group(index)) for index in range(1, 4))
    prerelease = match.group(4)
    return major, minor, patch, 1 if prerelease is None else 0, prerelease or ""


def is_newer_version(candidate: str, current: str = APP_VERSION) -> Optional[bool]:
    """Compare two app versions, or return None if either form is unsupported."""
    candidate_key = _version_key(candidate)
    current_key = _version_key(current)
    if candidate_key is None or current_key is None:
        return None
    return candidate_key > current_key


class GitHubReleaseChecker(QObject):
    """Perform an asynchronous, user-requested GitHub latest-release check."""

    def __init__(self, window: QMainWindow, app_name: str) -> None:
        super().__init__(window)
        self._window = window
        self._app_name = app_name
        self._manager = QNetworkAccessManager(self)
        self._reply: Optional[QNetworkReply] = None

    def check(self) -> None:
        api_url = github_latest_release_api_url()
        if not api_url:
            QMessageBox.information(
                self._window,
                "Check for Updates",
                "The public release repository has not been configured yet.\n\n"
                "Set DEFAULT_GITHUB_REPOSITORY in a2c_app_support.py when the "
                "public repository is created.",
            )
            return
        if self._reply is not None:
            return

        request = QNetworkRequest(QUrl(api_url))
        request.setRawHeader(b"Accept", b"application/vnd.github+json")
        request.setRawHeader(
            b"User-Agent", f"A2C-Sensor-Tools/{APP_VERSION}".encode("ascii")
        )
        request.setTransferTimeout(10_000)
        self._reply = self._manager.get(request)
        self._reply.finished.connect(self._finished)

    def _finished(self) -> None:
        reply = self._reply
        self._reply = None
        if reply is None:
            return
        try:
            if reply.error() != QNetworkReply.NetworkError.NoError:
                QMessageBox.warning(
                    self._window,
                    "Check for Updates",
                    f"Could not check for updates:\n{reply.errorString()}",
                )
                return
            try:
                release = json.loads(bytes(reply.readAll()).decode("utf-8"))
                tag = str(release["tag_name"])
                release_url = str(release["html_url"])
            except (KeyError, TypeError, ValueError, UnicodeDecodeError) as exc:
                QMessageBox.warning(
                    self._window,
                    "Check for Updates",
                    f"GitHub returned an invalid release response:\n{exc}",
                )
                return

            newer = is_newer_version(tag)
            if newer is None:
                QMessageBox.warning(
                    self._window,
                    "Check for Updates",
                    f"The latest release tag {tag!r} is not a supported version.\n"
                    "A2C Sensor Tools releases must use tags such as v1.2.3.",
                )
            elif newer:
                answer = QMessageBox.information(
                    self._window,
                    "Update Available",
                    f"{self._app_name} {tag} is available.\n"
                    f"This installation is version {APP_VERSION}.\n\n"
                    "Open the release download page?",
                    QMessageBox.StandardButton.Open | QMessageBox.StandardButton.Cancel,
                    QMessageBox.StandardButton.Open,
                )
                if answer == QMessageBox.StandardButton.Open:
                    QDesktopServices.openUrl(QUrl(release_url))
            else:
                QMessageBox.information(
                    self._window,
                    "Check for Updates",
                    f"You are running the latest version ({APP_VERSION}).",
                )
        finally:
            reply.deleteLater()


def show_about_dialog(window: QMainWindow, app_name: str) -> None:
    QMessageBox.about(
        window,
        f"About {app_name}",
        f"<h3>{app_name}</h3>"
        f"<p>Version {APP_VERSION}</p>"
        "<p>Copyright &copy; 2026 A2C.</p>"
        "<p><b>Open-source license</b></p>"
        "<p>Licensed under the "
        f'<a href="{APACHE_LICENSE_URL}">Apache License, Version 2.0</a> '
        '(the "License"); you may not use this software except in compliance '
        "with the License. Unless required by applicable law or agreed to in "
        "writing, software distributed under the License is distributed on an "
        '&quot;AS IS&quot; BASIS, WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND.</p>'
        "<p>This application uses Qt for Python (PySide6) and the Kvaser CANlib "
        "runtime. Third-party components remain subject to their own licenses.</p>"
        "<p>No update or telemetry request is made unless <b>Check for Updates</b> "
        "is selected by the user.</p>",
    )


def install_help_menu(window: QMainWindow, app_name: str) -> GitHubReleaseChecker:
    """Install a standard Help menu and return its retained update checker."""
    checker = GitHubReleaseChecker(window, app_name)
    help_menu = window.menuBar().addMenu("&Help")
    # Keep the Python wrapper alive. Some PySide6 versions can otherwise delete
    # a QMenu created by addMenu(str) after this function returns.
    setattr(window, "_a2c_help_menu", help_menu)

    update_action = QAction("Check for &Updates…", window)
    update_action.triggered.connect(checker.check)
    help_menu.addAction(update_action)

    project_url = github_project_url()
    if project_url:
        project_action = QAction("Open Project &Website", window)
        project_action.triggered.connect(
            lambda: QDesktopServices.openUrl(QUrl(project_url))
        )
        help_menu.addAction(project_action)

    help_menu.addSeparator()
    about_action = QAction(f"&About {app_name}", window)
    about_action.triggered.connect(lambda: show_about_dialog(window, app_name))
    help_menu.addAction(about_action)
    return checker
