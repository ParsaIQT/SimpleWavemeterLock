"""Use whichever Qt binding is installed: PySide6 (preferred), PyQt6 or PyQt5.

Set QT_API=pyside6|pyqt6|pyqt5 to force one. Code elsewhere uses fully
qualified enums (Qt.AlignmentFlag.AlignRight), which all three accept.
"""
import importlib
import os

_ORDER = ["PySide6", "PyQt6", "PyQt5"]
_forced = os.environ.get("QT_API", "").lower()
_ORDER.sort(key=lambda name: name.lower() != _forced)

for _name in _ORDER:
    try:
        QtCore = importlib.import_module(_name + ".QtCore")
        QtGui = importlib.import_module(_name + ".QtGui")
        QtWidgets = importlib.import_module(_name + ".QtWidgets")
        break
    except ImportError:
        continue
else:
    raise ImportError("no Qt binding found: pip install PySide6")

BINDING = _name
Signal = getattr(QtCore, "Signal", None) or QtCore.pyqtSignal
Qt = QtCore.Qt

if BINDING == "PyQt5":  # Qt6 does this by default
    QtWidgets.QApplication.setAttribute(Qt.ApplicationAttribute.AA_EnableHighDpiScaling, True)
    QtWidgets.QApplication.setAttribute(Qt.ApplicationAttribute.AA_UseHighDpiPixmaps, True)
