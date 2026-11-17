"""Drive the StructuraX desktop program on a private X display and take screenshots.

The program is an X11 client, so it can be run and looked at without touching the
desktop of the person at the machine:

    Xvfb :99 -screen 0 1600x1000x24 -nolisten tcp &
    DISPLAY=:99 WAYLAND_DISPLAY= target/debug/structurax gui &     # window 1440x900 at the origin
    XT_DISPLAY=:99 XT_OUT=/some/dir python3 -I scripts/gui_drive.py c:374,11 s:menu

Needs python-xlib (XTEST) and ImageMagick's `import`. The display is $XT_DISPLAY
(default :99); screenshots go to $XT_OUT (default: next to this script). There are
no modifier keys: select a field by triple click or clear it with `k:BackSpace`.
Forms re-lay out when a choice adds a row, so take a screenshot before clicking a
button whose position may have moved. Kill the program and Xvfb afterwards.

usage: gui_drive.py <step> [<step> ...] ; steps:
  c:X,Y        left click        r:X,Y   right click     m:X,Y  move
  d:X1,Y1,X2,Y2  left drag       t:TEXT  type text       k:KEYSYM  press a key (Return, Escape, F5, ...)
  w:SECONDS    wait              s:NAME  screenshot to NAME.png
"""
import os
import subprocess
import sys
import time

from Xlib import X, XK, display
from Xlib.ext import xtest

DISPLAY = os.environ.get("XT_DISPLAY", ":99")
HERE = os.environ.get("XT_OUT", os.path.dirname(os.path.abspath(__file__)))
d = display.Display(DISPLAY)
root = d.screen().root


def focus():
    kids = root.query_tree().children
    if kids:
        kids[-1].set_input_focus(X.RevertToParent, X.CurrentTime)
        d.sync()


def move(x, y):
    xtest.fake_input(d, X.MotionNotify, x=int(x), y=int(y))
    d.sync()
    time.sleep(0.08)


def button(n, x, y):
    move(x, y)
    xtest.fake_input(d, X.ButtonPress, n)
    d.sync()
    time.sleep(0.08)
    xtest.fake_input(d, X.ButtonRelease, n)
    d.sync()
    time.sleep(0.25)


def key(sym, shift=False):
    code = d.keysym_to_keycode(sym)
    if shift:
        xtest.fake_input(d, X.KeyPress, d.keysym_to_keycode(XK.XK_Shift_L))
    xtest.fake_input(d, X.KeyPress, code)
    xtest.fake_input(d, X.KeyRelease, code)
    if shift:
        xtest.fake_input(d, X.KeyRelease, d.keysym_to_keycode(XK.XK_Shift_L))
    d.sync()
    time.sleep(0.04)


NAMES = {" ": "space", ".": "period", ",": "comma", "-": "minus", "_": "underscore", "/": "slash",
         ":": "colon", ";": "semicolon", "=": "equal", "+": "plus", "(": "parenleft", ")": "parenright"}
SHIFTED = set('_:+()ABCDEFGHIJKLMNOPQRSTUVWXYZ')

focus()
for step in sys.argv[1:]:
    op, _, arg = step.partition(":")
    if op in ("c", "r", "m"):
        x, y = (float(v) for v in arg.split(","))
        if op == "m":
            move(x, y)
        else:
            button(1 if op == "c" else 3, x, y)
    elif op == "d":
        x1, y1, x2, y2 = (float(v) for v in arg.split(","))
        move(x1, y1)
        xtest.fake_input(d, X.ButtonPress, 1)
        d.sync()
        for i in range(1, 9):
            move(x1 + (x2 - x1) * i / 8, y1 + (y2 - y1) * i / 8)
        xtest.fake_input(d, X.ButtonRelease, 1)
        d.sync()
        time.sleep(0.25)
    elif op == "t":
        for ch in arg:
            name = NAMES.get(ch, ch)
            key(XK.string_to_keysym(name), ch in SHIFTED)
    elif op == "k":
        key(XK.string_to_keysym(arg))
        time.sleep(0.2)
    elif op == "w":
        time.sleep(float(arg))
    elif op == "s":
        time.sleep(0.4)
        subprocess.run(["import", "-display", DISPLAY, "-window", "root", os.path.join(HERE, arg + ".png")], check=True)
    else:
        raise SystemExit("unknown step " + step)
