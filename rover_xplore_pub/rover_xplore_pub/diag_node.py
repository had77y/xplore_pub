import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, Int32MultiArray

COL = 32  # largeur intérieure de chaque colonne


def _pad(s: str) -> str:
    return s[:COL].ljust(COL)


class DiagNode(Node):

    def __init__(self):
        super().__init__('diag_node')

        self._imu    = None
        self._enc    = None
        self._us     = None
        self._motors = None
        self._arm    = None
        self._lock   = threading.Lock()

        self.create_subscription(Imu,               '/imu/raw',              self._cb_imu,    10)
        self.create_subscription(Int32MultiArray,   '/wheel_encoders',       self._cb_enc,    10)
        self.create_subscription(Float32MultiArray, '/ultrasonic',           self._cb_us,     10)
        self.create_subscription(Int32MultiArray,   '/rover/motor_cmd',      self._cb_motors, 10)
        self.create_subscription(Int32MultiArray,   '/rover/arm_serial_cmd', self._cb_arm,    10)

        self.create_timer(0.1, self._print)

    def _cb_imu(self, msg: Imu):
        with self._lock: self._imu = msg

    def _cb_enc(self, msg: Int32MultiArray):
        with self._lock: self._enc = list(msg.data)

    def _cb_us(self, msg: Float32MultiArray):
        with self._lock: self._us = list(msg.data)

    def _cb_motors(self, msg: Int32MultiArray):
        with self._lock: self._motors = list(msg.data)

    def _cb_arm(self, msg: Int32MultiArray):
        with self._lock: self._arm = list(msg.data)

    def _build_columns(self, imu, enc, us, motors, arm):
        left, right = [], []

        # ── Colonne gauche : REÇU ──────────────────────────────────────────────

        left.append(_pad('── IMU ──────────────────────────'))
        if imu:
            left.append(_pad(f'  accel  ax = {imu.linear_acceleration.x:9.1f}'))
            left.append(_pad(f'         ay = {imu.linear_acceleration.y:9.1f}'))
            left.append(_pad(f'         az = {imu.linear_acceleration.z:9.1f}'))
            left.append(_pad(f'  gyro   gx = {imu.angular_velocity.x:9.1f}'))
            left.append(_pad(f'         gy = {imu.angular_velocity.y:9.1f}'))
            left.append(_pad(f'         gz = {imu.angular_velocity.z:9.1f}'))
        else:
            left += [_pad('  (en attente...)')] * 6

        left.append(_pad('── ENCODEURS ────────────────────'))
        if enc:
            m1, m2, m3, m4 = enc[:4]
            left.append(_pad(f'  FR = {m1:5d}    FL = {m2:5d}'))
            left.append(_pad(f'  BR = {m3:5d}    BL = {m4:5d}'))
        else:
            left += [_pad('  (en attente...)')] * 2

        left.append(_pad('── ULTRASONS (cm) ───────────────'))
        if us:
            v = [int(x) for x in us[:5]]
            left.append(_pad(f'  d1={v[0]:5d}  d2={v[1]:5d}  d3={v[2]:5d}'))
            left.append(_pad(f'  d4={v[3]:5d}  d5={v[4]:5d}'))
        else:
            left += [_pad('  (en attente...)')] * 2

        # ── Colonne droite : ENVOYÉ ────────────────────────────────────────────

        right.append(_pad('── MOTEURS ──────────────────────'))
        if motors:
            fr, fl, br, bl = motors[:4]
            right.append(_pad(f'  FR = {fr:5d}'))
            right.append(_pad(f'  FL = {fl:5d}'))
            right.append(_pad(f'  BR = {br:5d}'))
            right.append(_pad(f'  BL = {bl:5d}'))
        else:
            right += [_pad('  (en attente...)')] * 4

        right.append(_pad('── BRAS ─────────────────────────'))
        if arm:
            s1, s2, s3, s4, step = arm[:5]
            right.append(_pad(f'  servo 1  = {s1:5d}'))
            right.append(_pad(f'  servo 2  = {s2:5d}'))
            right.append(_pad(f'  servo 3  = {s3:5d}'))
            right.append(_pad(f'  servo 4  = {s4:5d}'))
            right.append(_pad(f'  stepper  = {step:5d}'))
        else:
            right += [_pad('  (en attente...)')] * 5

        # Égaliser les hauteurs
        n = max(len(left), len(right))
        left  += [_pad('')] * (n - len(left))
        right += [_pad('')] * (n - len(right))

        return left, right

    def _print(self):
        with self._lock:
            imu    = self._imu
            enc    = self._enc
            us     = self._us
            motors = self._motors
            arm    = self._arm

        left, right = self._build_columns(imu, enc, us, motors, arm)

        sep = '─' * COL
        print('\033[H\033[J', end='')
        print(f'╔{sep}╦{sep}╗')
        print(f'║{"← REÇU  (Arduino → Pi)":^{COL}}║{"ENVOYÉ →  (Pi → Arduino)":^{COL}}║')
        print(f'╠{sep}╬{sep}╣')
        for l, r in zip(left, right):
            print(f'║{l}║{r}║')
        print(f'╚{sep}╩{sep}╝')
        print(f'  {time.strftime("%H:%M:%S")}   Ctrl+C pour quitter')


def main(args=None):
    rclpy.init(args=args)
    node = DiagNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
