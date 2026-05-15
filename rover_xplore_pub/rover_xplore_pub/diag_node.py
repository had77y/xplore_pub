import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, Int32MultiArray


class DiagNode(Node):

    def __init__(self):
        super().__init__('diag_node')

        self._imu  = None
        self._enc  = None
        self._us   = None
        self._lock = threading.Lock()

        self.create_subscription(Imu,               '/imu/raw',        self._cb_imu, 10)
        self.create_subscription(Int32MultiArray,   '/wheel_encoders', self._cb_enc, 10)
        self.create_subscription(Float32MultiArray, '/ultrasonic',     self._cb_us,  10)

        self.create_timer(0.1, self._print)

    def _cb_imu(self, msg: Imu):
        with self._lock:
            self._imu = msg

    def _cb_enc(self, msg: Int32MultiArray):
        with self._lock:
            self._enc = list(msg.data)

    def _cb_us(self, msg: Float32MultiArray):
        with self._lock:
            self._us = list(msg.data)

    def _print(self):
        with self._lock:
            imu = self._imu
            enc = self._enc
            us  = self._us

        print('\033[H\033[J', end='')  # efface le terminal
        print('╔══════════════════════════════════════════════╗')
        print('║         XPLORE — Diagnostic Arduino          ║')
        print('╠══════════════════════════════════════════════╣')

        if imu:
            ax = imu.linear_acceleration.x
            ay = imu.linear_acceleration.y
            az = imu.linear_acceleration.z
            gx = imu.angular_velocity.x
            gy = imu.angular_velocity.y
            gz = imu.angular_velocity.z
            print(f'║  IMU  accel  ax={ax:8.1f}  ay={ay:8.1f}  az={az:8.1f}  ║')
            print(f'║         gyro  gx={gx:8.1f}  gy={gy:8.1f}  gz={gz:8.1f}  ║')
        else:
            print('║  IMU    (en attente...)                      ║')
            print('║                                              ║')

        print('╠══════════════════════════════════════════════╣')

        if enc:
            m1, m2, m3, m4 = enc[:4]
            print(f'║  ENC  m1={m1:5d}  m2={m2:5d}  m3={m3:5d}  m4={m4:5d}  ║')
        else:
            print('║  ENC  (en attente...)                        ║')

        print('╠══════════════════════════════════════════════╣')

        if us:
            vals = [int(v) for v in us[:5]]
            print(f'║  US   {vals[0]:4d}cm  {vals[1]:4d}cm  {vals[2]:4d}cm  {vals[3]:4d}cm  {vals[4]:4d}cm  ║')
        else:
            print('║  US   (en attente...)                        ║')

        print('╚══════════════════════════════════════════════╝')
        print(f'  mis à jour : {time.strftime("%H:%M:%S")}   Ctrl+C pour quitter')


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
