import threading
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32MultiArray, Int32MultiArray


class DiagNode(Node):

    def __init__(self):
        super().__init__('diag_node')

        # Reçu de l'Arduino
        self._imu  = None
        self._enc  = None
        self._us   = None
        # Envoyé à l'Arduino
        self._motors = None
        self._arm    = None

        self._lock = threading.Lock()

        # Topics reçus (Arduino → Pi → ROS2)
        self.create_subscription(Imu,               '/imu/raw',            self._cb_imu,    10)
        self.create_subscription(Int32MultiArray,   '/wheel_encoders',     self._cb_enc,    10)
        self.create_subscription(Float32MultiArray, '/ultrasonic',         self._cb_us,     10)
        # Topics envoyés (ROS2 → Pi → Arduino)
        self.create_subscription(Int32MultiArray,   '/rover/motor_cmd',      self._cb_motors, 10)
        self.create_subscription(Int32MultiArray,   '/rover/arm_serial_cmd', self._cb_arm,    10)

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

    def _cb_motors(self, msg: Int32MultiArray):
        with self._lock:
            self._motors = list(msg.data)

    def _cb_arm(self, msg: Int32MultiArray):
        with self._lock:
            self._arm = list(msg.data)

    def _print(self):
        with self._lock:
            imu    = self._imu
            enc    = self._enc
            us     = self._us
            motors = self._motors
            arm    = self._arm

        print('\033[H\033[J', end='')
        print('╔══════════════════════════════════════════════════╗')
        print('║           XPLORE — Diagnostic Arduino            ║')
        print('╠══════════════════════╦═══════════════════════════╣')
        print('║     ← REÇU           ║     ENVOYÉ →              ║')
        print('╠══════════════════════╬═══════════════════════════╣')

        # IMU
        if imu:
            ax = imu.linear_acceleration.x
            ay = imu.linear_acceleration.y
            az = imu.linear_acceleration.z
            gx = imu.angular_velocity.x
            gy = imu.angular_velocity.y
            gz = imu.angular_velocity.z
            print(f'║ IMU accel            ║                           ║')
            print(f'║  ax={ax:7.1f}           ║                           ║')
            print(f'║  ay={ay:7.1f}           ║                           ║')
            print(f'║  az={az:7.1f}           ║                           ║')
            print(f'║ gyro                 ║                           ║')
            print(f'║  gx={gx:7.1f}           ║                           ║')
            print(f'║  gy={gy:7.1f}           ║                           ║')
            print(f'║  gz={gz:7.1f}           ║                           ║')
        else:
            print('║ IMU  (en attente...) ║                           ║')
            for _ in range(7):
                print('║                      ║                           ║')

        print('╠══════════════════════╣                           ║')

        # Encodeurs
        if enc:
            m1, m2, m3, m4 = enc[:4]
            print(f'║ ENC                  ║                           ║')
            print(f'║  FR={m1:5d} FL={m2:5d}  ║                           ║')
            print(f'║  BR={m3:5d} BL={m4:5d}  ║                           ║')
        else:
            print('║ ENC  (en attente...) ║                           ║')
            print('║                      ║                           ║')
            print('║                      ║                           ║')

        print('╠══════════════════════╣                           ║')

        # Ultrasons
        if us:
            v = [int(x) for x in us[:5]]
            print(f'║ US (cm)              ║                           ║')
            print(f'║  {v[0]:4d} {v[1]:4d} {v[2]:4d}      ║                           ║')
            print(f'║  {v[3]:4d} {v[4]:4d}           ║                           ║')
        else:
            print('║ US   (en attente...) ║                           ║')
            print('║                      ║                           ║')
            print('║                      ║                           ║')

        print('╠══════════════════════╩═══════════════════════════╣')

        # Moteurs envoyés
        if motors:
            fr, fl, br, bl = motors[:4]
            print(f'║ MOTEURS  FR={fr:5d}  FL={fl:5d}  BR={br:5d}  BL={bl:5d}  ║')
        else:
            print('║ MOTEURS  (en attente...)                          ║')

        # Bras envoyé
        if arm:
            s1, s2, s3, s4, step = arm[:5]
            print(f'║ BRAS  s1={s1:5d} s2={s2:5d} s3={s3:5d} s4={s4:5d} st={step:5d} ║')
        else:
            print('║ BRAS     (en attente...)                          ║')

        print('╚══════════════════════════════════════════════════╝')
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
