IDEA: turns off auto white balance, so colors dont shift based on whats in the image
green looks like yellow
yellow looks like orange
orange looks like red
and blue just looks outright grey

but the important thing is that they can be distinguished

White balance
Run this before every session; the camera resets it:

bash
ros2 param set /v4l2_camera 'white_balance_auto_&_preset' 3
Colour values

Normal lighting (code defaults)	Night testing (runs 22–26)
rgb_tape	[120, 102, 95]	[140, 112, 95]
rgb_blue	[136, 134, 128]	[183, 174, 165]
rgb_green	[155, 140, 80]	[199, 182, 102]
rgb_yellow	[175, 126, 47]	[225, 166, 60]
rgb_orange	[190, 79, 57]	[241, 120, 84]
The defaults are what you get when you pass no rgb_* arguments. To use the night values:

bash
-p rgb_tape:="[140.0, 112.0, 95.0]" \
-p rgb_blue:="[183.0, 174.0, 165.0]" -p rgb_green:="[199.0, 182.0, 102.0]" \
-p rgb_yellow:="[225.0, 166.0, 60.0]" -p rgb_orange:="[241.0, 120.0, 84.0]"
Don't trust the "normal lighting" values without checking them. They come from an earlier session. They were already wrong this afternoon, before the sun went down: the tape read (153, 120, 105) in run 21, and with the default tape value the tape scored higher than blue. The safest option on demo day is to measure fresh values in that room:

bash
ros2 run final_project final_project --ros-args -p mode:=calibrate
Sit the robot still on the tape and on each of the four colours. Copy the CAL rgb=[...] values it prints into the rgb_* arguments.


ADDITIONAL VISUALIZER SCRIPT
#!/usr/bin/env python3
"""
Live view of the robot camera (/image_raw/compressed) in an OpenCV window.
Also prints the mean RGB of the whole frame on the image.

Overlays (to tell camera problems from control problems):
  red line      raw /line_idx (what the camera node says the tape is)
  green line    brightest column computed here from the same frame (cross-check)
  grey line     image centre (the PID setpoint)
  yellow arrow  /cmd_vel angular.z (left = turning left)
  top banner    what final_project is doing (/follow_state):
                  green TAPE (PID on line_idx) | magenta PATCH (driving blind, line ignored)
                  orange EXIT HOLD / REACQUIRE (blind, waiting for the line)
                  red IGNORING LINE (jump filter rejecting readings) | blue DELIVER
If red sits on the tape but the robot still steers wrong -> control.
If red is not on the tape (or jumps around while the tape is in view) -> camera.

Run:  python3 live_view.py        (q or Esc to quit, s to save a screenshot)
"""

import time

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, UInt32

LINE_W = 640        # width of the array line_idx indexes into (final_project uses center=320)


class LiveView(Node):

    def __init__(self):
        super().__init__('live_view')
        self.create_subscription(CompressedImage, '/image_raw/compressed',
                                 self.cb, qos_profile_sensor_data)
        self.create_subscription(UInt32, '/line_idx', self.line_cb, 10)
        self.create_subscription(TwistStamped, '/cmd_vel', self.cmd_cb, 10)
        self.create_subscription(String, '/follow_state', self.state_cb, 10)
        self.state = None
        self.state_t = None
        self.frame = None
        self.line = None
        self.line_t = None
        self.turn = None
        self.turn_t = None

    def line_cb(self, msg):
        self.line = int(msg.data)
        self.line_t = time.time()

    def state_cb(self, msg):
        self.state = msg.data
        self.state_t = time.time()

    def cmd_cb(self, msg):
        self.turn = msg.twist.angular.z
        self.turn_t = time.time()

    def cb(self, msg):
        img = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        b, g, r = img.reshape(-1, 3).mean(axis=0)
        h, w = img.shape[:2]
        sx = w / LINE_W
        now = time.time()

        # image centre (setpoint)
        cx = int(w / 2)
        cv2.line(img, (cx, 0), (cx, h), (160, 160, 160), 1)

        # own estimate: brightest column (column mean of grayscale), as a cross-check
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY).astype(np.float32)
        col = cv2.GaussianBlur(gray.mean(axis=0).reshape(1, -1), (1, 15), 0).ravel()
        own = int(np.argmax(col))
        cv2.line(img, (own, int(0.55 * h)), (own, h), (0, 200, 0), 2)

        # raw /line_idx from the camera node
        if self.line is not None:
            lx = int(self.line * sx)
            age = now - self.line_t
            colour = (0, 0, 255) if age < 0.5 else (0, 0, 120)     # dark red = stale
            cv2.line(img, (lx, 0), (lx, int(0.55 * h)), colour, 3)
            cv2.putText(img, f'line_idx={self.line} err={LINE_W / 2 - self.line:+.0f} '
                        f'age={age * 1000:.0f}ms', (10, 60),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, colour, 2)
        cv2.putText(img, f'own={own / sx:.0f}', (10, 85),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 0), 2)

        # commanded turn (+ = left)
        if self.turn is not None and now - self.turn_t < 0.5:
            tip = int(cx - 150 * self.turn)
            cv2.arrowedLine(img, (cx, h - 20), (tip, h - 20), (0, 220, 255), 3, tipLength=0.2)
            cv2.putText(img, f'turn={self.turn:+.2f}', (10, 110),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 220, 255), 2)

        cv2.putText(img, f'mean rgb=[{r:.0f}, {g:.0f}, {b:.0f}]', (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)

        # banner + frame border: what final_project is doing (BGR colours)
        if self.state is not None and now - self.state_t < 0.5:
            parts = self.state.split('|')
            mode = parts[0]
            colour = {'TAPE': (60, 180, 60), 'PATCH': (200, 60, 200),
                      'EXIT': (0, 140, 255), 'REACQUIRE': (0, 140, 255),
                      'IGNORING': (0, 0, 230), 'DELIVER': (230, 120, 0)}.get(
                          mode.split()[0], (120, 120, 120))
            border = 3 if mode == 'TAPE' else 12
            cv2.rectangle(img, (0, 0), (w - 1, h - 1), colour, border)
            bh = 34
            img = cv2.copyMakeBorder(img, bh, 0, 0, 0, cv2.BORDER_CONSTANT, value=colour)
            cv2.putText(img, mode, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 255), 2)
            cv2.putText(img, '  '.join(parts[1:]), (230, 24), cv2.FONT_HERSHEY_SIMPLEX,
                        0.5, (255, 255, 255), 1)
        else:
            img = cv2.copyMakeBorder(img, 34, 0, 0, 0, cv2.BORDER_CONSTANT, value=(60, 60, 60))
            cv2.putText(img, 'final_project not running (no /follow_state)', (10, 24),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
        self.frame = img


def main():
    rclpy.init()
    node = LiveView()
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.05)
            if node.frame is not None:
                cv2.imshow('robot camera', node.frame)
            key = cv2.waitKey(1) & 0xFF
            if key in (ord('q'), 27):
                break
            if key == ord('s') and node.frame is not None:
                cv2.imwrite(f'live_{time.time():.0f}.png', node.frame)
    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
