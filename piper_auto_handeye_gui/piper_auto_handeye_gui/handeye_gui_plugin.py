#!/usr/bin/env python3
"""RQt plugin wrapper for the Piper hand-eye calibration GUI.

rqt identifies this plugin by its FULL class path, so launch files use:

    rqt --force-discover --standalone \
        piper_auto_handeye_gui.handeye_gui_plugin.HandeyeGuiPlugin

The plugin itself owns no ROS logic -- it hands ``context.node`` (the node rqt
already spins) to :class:`HandeyeGuiWidget`, which talks to the calibration
stack purely over topics/services/actions. Closing the GUI therefore never
stops a running calibration.
"""

from rqt_gui_py.plugin import Plugin

from .handeye_gui_widget import HandeyeGuiWidget


class HandeyeGuiPlugin(Plugin):
    def __init__(self, context):
        super().__init__(context)
        self.setObjectName("HandeyeGuiPlugin")

        self._widget = HandeyeGuiWidget(context.node)
        if context.serial_number() > 1:
            self._widget.setWindowTitle(
                f"{self._widget.windowTitle()} ({context.serial_number()})")
        context.add_widget(self._widget)

    def shutdown_plugin(self):
        self._widget.shutdown()

    def save_settings(self, plugin_settings, instance_settings):
        self._widget.save_settings(instance_settings)

    def restore_settings(self, plugin_settings, instance_settings):
        self._widget.restore_settings(instance_settings)
