#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
robot_settings_app.py — Kivy 設定頁（獨立節點）
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool as RosBool, String as RosString

from kivy.app import App
from kivy.uix.floatlayout import FloatLayout
from kivy.uix.button import Button
from kivy.uix.label import Label
from kivy.graphics import Color, Rectangle


class SettingsRosNode(Node):
    def __init__(self):
        super().__init__("robot_settings_ui")
        self.pub_setting_mode = self.create_publisher(RosBool, "system/setting_mode", 10)
        self.pub_reload       = self.create_publisher(RosBool, "system/reload_config", 10)
        self.pub_network      = self.create_publisher(RosString, "system/network_settings", 10)

    def set_mode(self, flag: bool):
        self.pub_setting_mode.publish(RosBool(data=flag))

    def reload(self):
        self.pub_reload.publish(RosBool(data=True))

    def send_network(self, payload: str):
        self.pub_network.publish(RosString(data=payload))


class SettingView(FloatLayout):
    def __init__(self, ros: SettingsRosNode | None, **kwargs):
        super().__init__(**kwargs)
        self.ros = ros
        self.size_hint = (1, 1)
        with self.canvas.before:
            Color(0, 0, 0, 0.8)
            self.bg = Rectangle(pos=self.pos, size=self.size)
        self.bind(pos=self._upd, size=self._upd)
        self._build_menu()

    def attach_ros(self, ros: SettingsRosNode):
        self.ros = ros

    def _upd(self, *args):
        self.bg.pos = self.pos
        self.bg.size = self.size

    def _build_menu(self):
        self.clear_widgets()
        buttons = [
            ("返回",       {'center_x': 0.5, 'center_y': 0.8}, self._cb_return),
            ("重載配置",   {'center_x': 0.5, 'center_y': 0.6}, self._cb_reload),
            ("網路設定",   {'center_x': 0.5, 'center_y': 0.4}, self._cb_network),
            ("開發者模式", {'center_x': 0.5, 'center_y': 0.2}, self._cb_dev),
        ]
        for text, pos_hint, cb in buttons:
            btn = Button(text=text, size_hint=(None, None), size=(220, 220),
                         pos_hint=pos_hint, background_normal='', background_color=(0.2, 0.6, 0.8, 1))
            btn.bind(on_press=lambda *_ , f=cb: f())
            self.add_widget(btn)

    def _cb_return(self):
        if self.ros:
            self.ros.set_mode(False)
        App.get_running_app().stop()

    def _cb_reload(self):
        if self.ros:
            self.ros.reload()
            self.ros.set_mode(False)
        App.get_running_app().stop()

    def _cb_network(self):
        self.clear_widgets()
        info = Label(text="網路設定開發中\n未來列出 Wi-Fi 並可輸入密碼",
                     halign='center', valign='middle')
        info.bind(size=lambda inst, val: setattr(inst, 'text_size', val))
        self.add_widget(info)
        back = Button(text="返回設定", size_hint=(0.3, 0.1), pos_hint={'center_x': 0.5, 'y': 0.1})
        back.bind(on_press=lambda *_: self._build_menu())
        self.add_widget(back)

    def _cb_dev(self):
        App.get_running_app().stop()


class RobotSettingsApp(App):
    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        self.root_widget = SettingView(ros=None)   # 先不建 node
        self.node: SettingsRosNode | None = None

    def build(self):
        return self.root_widget

    def on_start(self):
        if not rclpy.ok():
            rclpy.init(args=None)
        self.node = SettingsRosNode()
        self.root_widget.attach_ros(self.node)

        from kivy.clock import Clock
        Clock.schedule_interval(lambda dt: rclpy.spin_once(self.node, timeout_sec=0.0), 0.02)

    def on_stop(self):
        try:
            if self.node is not None:
                self.node.destroy_node()
        finally:
            if rclpy.ok():
                rclpy.shutdown()


def main():
    RobotSettingsApp().run()


if __name__ == "__main__":
    main()
