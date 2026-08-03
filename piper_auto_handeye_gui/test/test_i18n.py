#!/usr/bin/env python3
"""The English GUI must be fully English.

tr() falls through untranslated strings on purpose, so a missing entry is
invisible at runtime -- it just shows that one label in Korean while everything
around it is English. These tests read the widget source and check the two
sides actually line up, which is the only place that mismatch can be caught
without a display attached.
"""

import ast
import os
import re

import pytest

HERE = os.path.dirname(__file__)
PKG = os.path.join(HERE, "..", "piper_auto_handeye_gui")
WIDGET = os.path.join(PKG, "handeye_gui_widget.py")


def _tr_literals():
    """Every constant string passed to tr() in the widget."""
    tree = ast.parse(open(WIDGET, encoding="utf-8").read())
    out = []
    for node in ast.walk(tree):
        if (isinstance(node, ast.Call)
                and isinstance(node.func, ast.Name) and node.func.id == "tr"
                and node.args and isinstance(node.args[0], ast.Constant)
                and isinstance(node.args[0].value, str)):
            out.append(node.args[0].value)
    return out


@pytest.fixture(scope="module")
def en():
    import importlib
    import piper_auto_handeye_gui.i18n as i18n
    importlib.reload(i18n)
    return i18n._EN


def test_widget_actually_calls_tr():
    """Guard the guard: an empty literal list would make everything below pass."""
    assert len(_tr_literals()) > 50


def test_every_tr_string_has_an_english_entry(en):
    missing = sorted(set(_tr_literals()) - set(en))
    assert not missing, ("no English text for:\n  "
                         + "\n  ".join(repr(m) for m in missing))


def test_no_stale_translations(en):
    """Entries for text the widget no longer shows are dead weight."""
    unused = sorted(set(en) - set(_tr_literals()))
    assert not unused, ("translated but never used:\n  "
                        + "\n  ".join(repr(u) for u in unused))


def test_format_placeholders_match(en):
    """A translation with the wrong number of {} holes raises at format() time.

    That crash would land in a ROS callback during a live run, so check the
    counts here instead of discovering it when the arm is moving.
    """
    for ko, en_text in en.items():
        assert ko.count("{}") == en_text.count("{}"), \
            f"placeholder count differs:\n  ko: {ko!r}\n  en: {en_text!r}"


def test_english_translations_are_not_korean(en):
    """Catch a Korean string pasted into the English column by accident."""
    for ko, en_text in en.items():
        assert not re.search(r"[가-힣]", en_text), \
            f"English entry still contains Korean: {en_text!r} (for {ko!r})"


def test_default_language_is_korean(monkeypatch):
    import importlib
    monkeypatch.delenv("HANDEYE_GUI_LANG", raising=False)
    import piper_auto_handeye_gui.i18n as i18n
    importlib.reload(i18n)
    assert i18n.LANG == "ko"
    assert i18n.tr("CAN 연결") == "CAN 연결"


def test_english_is_selected_by_env(monkeypatch):
    import importlib
    monkeypatch.setenv("HANDEYE_GUI_LANG", "en")
    import piper_auto_handeye_gui.i18n as i18n
    importlib.reload(i18n)
    assert i18n.LANG == "en"
    assert i18n.tr("CAN 연결") == "Connect CAN"
    # unknown text falls through rather than raising
    assert i18n.tr("한글 미등록") == "한글 미등록"
    importlib.reload(i18n)


def test_unknown_language_falls_back_to_korean(monkeypatch):
    import importlib
    monkeypatch.setenv("HANDEYE_GUI_LANG", "fr")
    import piper_auto_handeye_gui.i18n as i18n
    importlib.reload(i18n)
    assert i18n.LANG == "ko"
