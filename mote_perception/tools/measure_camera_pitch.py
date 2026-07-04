"""Measure the camera's pitch/roll relative to the floor it is sitting on.

Place the calibration checkerboard flat on the floor in the camera's view and run
this. It detects the board, solves its pose with the live /camera_info intrinsics,
and reads the floor plane's orientation in the camera frame -- giving the camera's
pitch (nose-down positive), roll, and height above the floor. Because the board
defines the actual floor patch in front of the robot, the result folds in any
chassis tilt at rest and the local floor slope, which an inclinometer-vs-gravity
reading would not.

Run where it can see the camera topics (on the Pi, or a workstation sharing the
graph):
    pixi run python mote_perception/tools/measure_camera_pitch.py

Run it twice with the board at two distances and compare: a stable pitch means the
floor patch is flat; a drifting one means it isn't.
"""

import argparse

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, CompressedImage


class PitchMeasurer(Node):
    def __init__(self, pattern, square, frames, image_topic, info_topic):
        super().__init__("measure_camera_pitch")
        self.pattern = pattern  # (inner corners across, down)
        self.square = square
        self.target = frames
        cols, rows = pattern
        objp = np.zeros((cols * rows, 3), np.float32)
        objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2) * square
        self.objp = objp

        self.K = None
        self.D = None
        self.samples = []  # (pitch_deg, roll_deg, height_m, reproj_px)
        self.done = False

        self.create_subscription(CameraInfo, info_topic, self._on_info, 10)
        self.create_subscription(CompressedImage, image_topic, self._on_image, 5)
        self.get_logger().info(
            f"waiting for board {cols}x{rows} ({square * 1000:.0f} mm) + camera_info"
        )

    def _on_info(self, msg):
        self.K = np.asarray(msg.k, np.float64).reshape(3, 3)
        self.D = np.asarray(msg.d, np.float64).reshape(-1)

    def _on_image(self, msg):
        if self.K is None or self.done:
            return
        img = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            return
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, self.pattern)
        if not found:
            return
        cv2.cornerSubPix(
            gray,
            corners,
            (11, 11),
            (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001),
        )
        ok, rvec, tvec = cv2.solvePnP(self.objp, corners, self.K, self.D)
        if not ok:
            return

        proj, _ = cv2.projectPoints(self.objp, rvec, tvec, self.K, self.D)
        reproj = float(
            np.sqrt(((proj.reshape(-1, 2) - corners.reshape(-1, 2)) ** 2).sum(1).mean())
        )

        R, _ = cv2.Rodrigues(rvec)
        n = R[:, 2] / np.linalg.norm(R[:, 2])  # board normal in optical frame
        if n[1] > 0:  # force it to point up (optical +y is down)
            n = -n
        # Optical frame: x right, y down, z forward. Up-normal of a level camera is
        # (0,-1,0); a nose-down pitch tips it toward -z, a roll toward -x.
        pitch = np.degrees(np.arctan2(-n[2], -n[1]))
        roll = np.degrees(np.arctan2(-n[0], -n[1]))
        height = abs(float(n @ tvec.reshape(3)))

        self.samples.append((pitch, roll, height, reproj))
        self.get_logger().info(
            f"[{len(self.samples)}/{self.target}] pitch {pitch:+.2f} deg  "
            f"roll {roll:+.2f} deg  height {height:.3f} m  reproj {reproj:.2f} px"
        )
        if len(self.samples) >= self.target:
            self.done = True

    def report(self):
        if not self.samples:
            self.get_logger().error("no board detections -- check placement/lighting")
            return
        a = np.array(self.samples)
        pitch, roll, height, reproj = a[:, 0], a[:, 1], a[:, 2], a[:, 3]
        print("\n=== camera pose relative to the floor ===")
        print(f"  samples:  {len(a)}  (mean reproj {reproj.mean():.2f} px)")
        print(
            f"  pitch :  {pitch.mean():+.2f} deg  (std {pitch.std():.2f})  [nose-down positive]"
        )
        print(f"  roll  :  {roll.mean():+.2f} deg  (std {roll.std():.2f})")
        print(f"  height:  {height.mean():.3f} m   (std {height.std():.3f})")
        if reproj.mean() > 1.0:
            print(
                "  ! high reprojection error -- detections are noisy, treat with care"
            )
        print(
            f"\n  to model this static tilt: camera_joint rpy "
            f'"0 {np.radians(pitch.mean()):.4f} 0"  (verify against the full-cloud view)'
        )


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--size", default="6x9", help="inner corners, e.g. 6x9")
    ap.add_argument("--square", type=float, default=0.025, help="square size (m)")
    ap.add_argument("--frames", type=int, default=15, help="good detections to average")
    ap.add_argument(
        "--image", default="/image_raw/compressed", help="compressed image topic"
    )
    ap.add_argument("--info", default="/camera_info", help="camera_info topic")
    args = ap.parse_args()
    cols, rows = (int(v) for v in args.size.lower().split("x"))

    rclpy.init()
    node = PitchMeasurer((cols, rows), args.square, args.frames, args.image, args.info)
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    node.report()
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
