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

"""
Graphical-user interface code for LaneZero
"""

import sys

from ._viewer_core import enable

if enable:
    from PySide6.QtGui import QAction

__all__ = [
    'controller',
    'launch',
]


def launch():
    return controller.launch()


class _Singleton(type):
    _instances = {}

    def __call__(cls, *args, **kw):
        if cls not in cls._instances:
            cls._instances[cls] = super(_Singleton, cls).__call__(*args, **kw)
        return cls._instances[cls]


class _Controller(metaclass=_Singleton):
    def __init__(self):
        self._rmgr = None

    def __getattr__(self, name):
        return None if self._rmgr is None else getattr(self._rmgr, name)

    def launch(self, name="LaneZero Viewer", size=(1200, 800)):
        from ._viewer_core import RManager
        self._rmgr = RManager.get_instance()
        self._rmgr.set_up()
        self._rmgr.set_window_title(name)
        self._rmgr.resize(w=size[0], h=size[1])
        self.populate_menu()
        self._rmgr.show()
        return self._rmgr.exec()

    def populate_menu(self):
        wm = self._rmgr

        def _add_action(menu, text, tip, func, checkable=False, checked=False):
            act = QAction(text, wm.main_window)
            act.setStatusTip(tip)
            act.setCheckable(checkable)
            if checkable:
                act.setChecked(checked)
            if callable(func):
                act.triggered.connect(lambda *a: func())
            menu.addAction(act)

        if sys.platform != 'darwin':
            _add_action(
                menu=wm.file_menu,
                text="Exit",
                tip="Exit the application",
                func=lambda: wm.quit(),
            )


controller = _Controller()

# vim: set ff=unix fenc=utf8 et sw=4 ts=4 sts=4 tw=79:
