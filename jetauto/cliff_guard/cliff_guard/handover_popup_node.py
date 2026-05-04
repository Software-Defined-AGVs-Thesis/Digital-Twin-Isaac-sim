#!/usr/bin/env python3
"""
handover_popup_node.py — Human-in-the-loop popup for cliff recovery.

Subscribes to /cliff_guard/status (latched). When emergency_stop flips True
a Tk dialog pops up asking the human to take manual control. Accept calls
/vr_override/hold (sticky) — Nav2 stays paused while the human drives the
robot using whatever teleop they prefer (VR controller, teleop_twist_keyboard
window, etc.). Resume Autonomous calls /vr_override/release then
/cliff_guard/resume_nav. If the resume is rejected (cliff still visible or
depth stale) the panel stays open with the rejection message and
re-engages hold so Nav2 doesn't silently come back.

The popup tracks /vr_override/active to confirm release actually happened
(rather than trusting only the service response), so the UI never lies.

If $DISPLAY is unset or tkinter import fails, the node logs a warning and
exits cleanly so the rest of the launch is unaffected.
"""

import os
import sys
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


def _have_display():
    return bool(os.environ.get('DISPLAY'))


class HandoverPopup(Node):
    def __init__(self, ui):
        super().__init__('handover_popup')
        self.ui = ui

        latched_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
        )

        self.sub_status = self.create_subscription(
            Bool, '/cliff_guard/status', self._status_cb, latched_qos)
        self.sub_alert = self.create_subscription(
            String, '/cliff_guard/alert', self._alert_cb, latched_qos)
        self.sub_vr_active = self.create_subscription(
            Bool, '/vr_override/active', self._vr_active_cb, latched_qos)

        self.cli_hold = self.create_client(Trigger, '/vr_override/hold')
        self.cli_release = self.create_client(Trigger, '/vr_override/release')
        self.cli_resume_nav = self.create_client(Trigger, '/cliff_guard/resume_nav')

        self.get_logger().info('handover_popup ready — listening for /cliff_guard/status.')

    def _status_cb(self, msg):
        self.ui.schedule(self.ui.on_status, msg.data)

    def _alert_cb(self, msg):
        self.ui.schedule(self.ui.on_alert, msg.data)

    def _vr_active_cb(self, msg):
        self.ui.schedule(self.ui.on_vr_active, msg.data)

    def call_async_trigger(self, client, on_done):
        """Fire a Trigger service and call on_done(success, message) on Tk thread."""
        if not client.service_is_ready():
            ready = client.wait_for_service(timeout_sec=0.5)
            if not ready:
                self.ui.schedule(on_done, False, f'Service {client.srv_name} not available.')
                return
        future = client.call_async(Trigger.Request())

        def poll():
            if future.done():
                try:
                    res = future.result()
                    self.ui.schedule(on_done, bool(res.success), res.message or '')
                except Exception as e:
                    self.ui.schedule(on_done, False, f'Service call failed: {e}')
            else:
                self.ui.root.after(50, poll)

        self.ui.root.after(50, poll)


class HandoverUI:
    """Tk UI state machine. All methods that touch Tk widgets must run on the
    Tk thread; ROS callbacks reach the UI via `schedule()`."""

    STATE_IDLE = 'idle'
    STATE_ALERT = 'alert'      # popup asking Accept/Dismiss
    STATE_ACTIVE = 'active'    # human in control, sticky_hold engaged
    STATE_RESUMING = 'resuming'  # waiting for release + resume_nav

    def __init__(self):
        import tkinter as tk
        self.tk = tk
        self.root = tk.Tk()
        self.root.title('OmniLRS Cliff Handover')
        self.root.geometry('440x300')
        self.root.protocol('WM_DELETE_WINDOW', self._on_close_request)

        self.state = self.STATE_IDLE
        self.last_status = False
        self.last_alert = ''
        self.vr_active = False
        self.node = None  # set by main()

        self.frame_idle = tk.Frame(self.root)
        self.frame_alert = tk.Frame(self.root)
        self.frame_active = tk.Frame(self.root)

        self._build_idle()
        self._build_alert()
        self._build_active()

        self._show_frame(self.frame_idle)

    # ---------- thread bridge ----------
    def schedule(self, fn, *args):
        self.root.after(0, fn, *args)

    # ---------- UI construction ----------
    def _build_idle(self):
        tk = self.tk
        f = self.frame_idle
        tk.Label(f, text='Cliff Handover Monitor',
                 font=('Helvetica', 16, 'bold')).pack(pady=20)
        tk.Label(f, text='Monitoring /cliff_guard/status…',
                 font=('Helvetica', 11)).pack(pady=4)
        self.lbl_idle_alert = tk.Label(f, text='', wraplength=400, fg='gray30',
                                       font=('Helvetica', 9), justify='left')
        self.lbl_idle_alert.pack(pady=8, padx=10)

    def _build_alert(self):
        tk = self.tk
        f = self.frame_alert
        tk.Label(f, text='⚠  CLIFF DETECTED', font=('Helvetica', 18, 'bold'),
                 fg='red').pack(pady=12)
        self.lbl_alert_msg = tk.Label(f, text='', wraplength=400,
                                      font=('Helvetica', 10), justify='left')
        self.lbl_alert_msg.pack(pady=6, padx=10)
        tk.Label(f, text='Robot is halted and Nav2 is paused.\n'
                          'Take manual control to drive away from the edge?',
                 font=('Helvetica', 11)).pack(pady=10)
        btns = tk.Frame(f); btns.pack(pady=14)
        tk.Button(btns, text='Accept (take control)', width=22,
                  bg='#2a8f3f', fg='white',
                  command=self._on_accept).pack(side='left', padx=8)
        tk.Button(btns, text='Dismiss', width=12,
                  command=self._on_dismiss).pack(side='left', padx=8)

    def _build_active(self):
        tk = self.tk
        f = self.frame_active
        tk.Label(f, text='MANUAL CONTROL ACTIVE',
                 font=('Helvetica', 14, 'bold'), fg='#2a8f3f').pack(pady=14)
        tk.Label(f, text='Drive the robot to safety using your usual teleop\n'
                          '(VR controller or teleop_twist_keyboard window).\n'
                          'Cliff guard is muted while you drive.',
                 font=('Helvetica', 10), justify='center').pack(pady=8)

        self.lbl_active_status = tk.Label(f, text='', wraplength=400,
                                          font=('Helvetica', 9), fg='gray30',
                                          justify='left')
        self.lbl_active_status.pack(pady=8, padx=10)

        self.btn_resume = tk.Button(f, text='Resume Autonomous (when safe)',
                                    width=30, bg='#1f5fa8', fg='white',
                                    command=self._on_resume_autonomous)
        self.btn_resume.pack(pady=10)

    def _show_frame(self, frame):
        for fr in (self.frame_idle, self.frame_alert, self.frame_active):
            fr.pack_forget()
        frame.pack(fill='both', expand=True)

    # ---------- ROS-driven state changes (run on Tk thread) ----------
    def on_status(self, status):
        self.last_status = status
        if status:
            if self.state == self.STATE_IDLE:
                self.lbl_alert_msg.config(text=self.last_alert or
                                          'Cliff detected by depth sensor.')
                self.state = self.STATE_ALERT
                self._show_frame(self.frame_alert)
                self.root.deiconify()
                self.root.lift()
                self.root.focus_force()
        else:
            if self.state != self.STATE_ACTIVE:
                self.state = self.STATE_IDLE
                self._show_frame(self.frame_idle)

    def on_alert(self, text):
        self.last_alert = text
        if self.state == self.STATE_ALERT:
            self.lbl_alert_msg.config(text=text)
        elif self.state == self.STATE_IDLE:
            self.lbl_idle_alert.config(text=text)
        elif self.state in (self.STATE_ACTIVE, self.STATE_RESUMING):
            self.lbl_active_status.config(text=text)

    def on_vr_active(self, active):
        # Authoritative truth for whether vr_override is engaged. Used to
        # confirm release actually happened.
        was_active = self.vr_active
        self.vr_active = active
        if self.state == self.STATE_RESUMING and was_active and not active:
            # Release confirmed by the latched topic — proceed to resume_nav.
            self.lbl_active_status.config(
                text='Override released. Asking cliff_guard to resume Nav2…')
            if self.node is not None:
                self.node.call_async_trigger(self.node.cli_resume_nav,
                                             self._after_resume_nav)

    # ---------- Button handlers ----------
    def _on_accept(self):
        if self.node is None:
            return
        self.lbl_active_status.config(text='Engaging override…')
        self.state = self.STATE_ACTIVE
        self._show_frame(self.frame_active)
        self.btn_resume.config(state='normal')
        self.node.call_async_trigger(self.node.cli_hold, self._after_hold)

    def _after_hold(self, ok, message):
        if ok:
            self.lbl_active_status.config(
                text='Override engaged. Drive to safety, then click Resume Autonomous.')
        else:
            self.lbl_active_status.config(text=f'WARNING: hold service failed — {message}')

    def _on_dismiss(self):
        self.state = self.STATE_IDLE
        self._show_frame(self.frame_idle)

    def _on_resume_autonomous(self):
        if self.node is None or self.state == self.STATE_RESUMING:
            return
        self.state = self.STATE_RESUMING
        self.btn_resume.config(state='disabled')
        self.lbl_active_status.config(text='Releasing override…')
        # Fire release; the actual progression to resume_nav happens in
        # on_vr_active() once /vr_override/active flips False, so the UI
        # reflects the real state of the world rather than just the
        # service-response optimism. We still attach a service-response
        # handler as a fallback in case the topic update is lost.
        self.node.call_async_trigger(self.node.cli_release, self._after_release)
        # Hard timeout — if neither the service response nor the topic
        # confirms within 5s, surface that to the user.
        self.root.after(5000, self._release_watchdog)

    def _release_watchdog(self):
        if self.state == self.STATE_RESUMING and self.vr_active:
            self.lbl_active_status.config(
                text='Release is taking longer than expected — still waiting…\n'
                     'You can click Resume Autonomous again to retry.')
            self.btn_resume.config(state='normal')

    def _after_release(self, ok, message):
        if not ok:
            self.lbl_active_status.config(text=f'Release failed — {message}')
            self.state = self.STATE_ACTIVE
            self.btn_resume.config(state='normal')
            return
        # If the topic already saw active=False before the service response
        # got back, on_vr_active already kicked off resume_nav. Otherwise the
        # message just confirms the service accepted the request — keep
        # waiting on the topic.
        if not self.vr_active and self.state == self.STATE_RESUMING:
            # on_vr_active didn't fire (topic was already False) — proceed.
            self.lbl_active_status.config(
                text='Override released. Asking cliff_guard to resume Nav2…')
            self.node.call_async_trigger(self.node.cli_resume_nav,
                                         self._after_resume_nav)

    def _after_resume_nav(self, ok, message):
        if ok:
            self.lbl_active_status.config(text=f'Resumed: {message}')
            self.state = self.STATE_IDLE
            self._show_frame(self.frame_idle)
            self.lbl_idle_alert.config(text=message)
        else:
            # Resume rejected (cliff still visible / depth stale). Re-engage
            # hold so the human can keep driving without Nav2 secretly waking up.
            self.lbl_active_status.config(
                text=f'Resume rejected: {message}\nRe-engaging hold — keep driving, then try again.')
            self.state = self.STATE_ACTIVE
            self.btn_resume.config(state='normal')
            self.node.call_async_trigger(self.node.cli_hold, lambda *_: None)

    def _on_close_request(self):
        # Don't allow closing the window during a handover.
        self.root.iconify()


def main(args=None):
    if not _have_display():
        print('[handover_popup] $DISPLAY not set — popup disabled.', file=sys.stderr)
        return 0

    try:
        import tkinter  # noqa: F401
    except Exception as e:
        print(f'[handover_popup] tkinter import failed ({e}) — popup disabled.',
              file=sys.stderr)
        return 0

    rclpy.init(args=args)

    ui = HandoverUI()
    node = HandoverPopup(ui)
    ui.node = node

    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()

    try:
        ui.root.mainloop()
    finally:
        try:
            node.destroy_node()
        finally:
            rclpy.shutdown()


if __name__ == '__main__':
    main()
