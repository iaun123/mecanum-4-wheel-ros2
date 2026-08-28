#!/usr/bin/env python3
import sys
import select
import termios
import tty
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

BANNER = """
==========================================================
 🎮 Mecanum Robot Keyboard Control (WASD)
==========================================================
 [การควบคุมการเคลื่อนที่ / Movement Controls]
        W
   A    S    D

   W : เดินหน้า (Forward)
   S : ถอยหลัง (Backward)
   A : เลี้ยวซ้าย (Turn Left)
   D : เลี้ยวขวา (Turn Right)

 [การสไลด์ข้าง Mecanum / Holonomic Strafing]
   Q : สไลด์ไปทางซ้าย (Strafe Left)
   E : สไลด์ไปทางขวา (Strafe Right)

 [การปรับความเร็ว / Speed Control]
   + / = : เพิ่มความเร็วเชิงเส้น (+0.05 m/s)
   - / _ : ลดความเร็วเชิงเส้น (-0.05 m/s)
   ]     : เพิ่มความเร็วเชิงมุม (+0.1 rad/s)
   [     : ลดความเร็วเชิงมุม (-0.1 rad/s)

 [ปุ่มอื่นๆ / Other Keys]
   SPACE หรือ X : หยุดหุ่นยนต์ทันที (Emergency Stop)
   M            : สลับโหมด A/D (เลี้ยว / สไลด์ข้าง)
   CTRL + C     : ออกจากโปรแกรม (Quit)
==========================================================
"""

class KeyboardTeleop(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.speed_linear = 0.3   # m/s
        self.speed_angular = 0.8  # rad/s
        self.strafe_mode = False  # False = A/D turns, True = A/D strafes

    def publish_twist(self, x=0.0, y=0.0, z=0.0):
        twist = Twist()
        twist.linear.x = float(x)
        twist.linear.y = float(y)
        twist.linear.z = 0.0
        twist.angular.x = 0.0
        twist.angular.y = 0.0
        twist.angular.z = float(z)
        self.publisher_.publish(twist)

def get_key(settings, timeout=0.1):
    tty.setraw(sys.stdin.fileno())
    rlist, _, _ = select.select([sys.stdin], [], [], timeout)
    if rlist:
        key = sys.stdin.read(1)
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    teleop = KeyboardTeleop()
    
    settings = termios.tcgetattr(sys.stdin)
    print(BANNER)
    
    try:
        linear_x = 0.0
        linear_y = 0.0
        angular_z = 0.0
        
        print(f"\rความเร็วปัจจุบัน: Linear={teleop.speed_linear:.2f} m/s | Angular={teleop.speed_angular:.2f} rad/s | โหมด A/D: {'[สไลด์ข้าง]' if teleop.strafe_mode else '[เลี้ยวซ้าย-ขวา]'}", end='', flush=True)
        
        while rclpy.ok():
            key = get_key(settings, timeout=0.1)
            
            if key:
                key_lower = key.lower()
                
                if key_lower == 'w':
                    linear_x = teleop.speed_linear
                    linear_y = 0.0
                    angular_z = 0.0
                    action = "เดินหน้า (Forward)"
                elif key_lower == 's':
                    linear_x = -teleop.speed_linear
                    linear_y = 0.0
                    angular_z = 0.0
                    action = "ถอยหลัง (Backward)"
                elif key_lower == 'a':
                    if teleop.strafe_mode:
                        linear_x = 0.0
                        linear_y = teleop.speed_linear
                        angular_z = 0.0
                        action = "สไลด์ซ้าย (Strafe Left)"
                    else:
                        linear_x = 0.0
                        linear_y = 0.0
                        angular_z = teleop.speed_angular
                        action = "เลี้ยวซ้าย (Turn Left)"
                elif key_lower == 'd':
                    if teleop.strafe_mode:
                        linear_x = 0.0
                        linear_y = -teleop.speed_linear
                        angular_z = 0.0
                        action = "สไลด์ขวา (Strafe Right)"
                    else:
                        linear_x = 0.0
                        linear_y = 0.0
                        angular_z = -teleop.speed_angular
                        action = "เลี้ยวขวา (Turn Right)"
                elif key_lower == 'q':
                    linear_x = 0.0
                    linear_y = teleop.speed_linear
                    angular_z = 0.0
                    action = "สไลด์ซ้าย (Strafe Left)"
                elif key_lower == 'e':
                    linear_x = 0.0
                    linear_y = -teleop.speed_linear
                    angular_z = 0.0
                    action = "สไลด์ขวา (Strafe Right)"
                elif key_lower in [' ', 'x']:
                    linear_x = 0.0
                    linear_y = 0.0
                    angular_z = 0.0
                    action = "หยุด (STOP)"
                elif key == '+' or key == '=':
                    teleop.speed_linear = min(2.0, teleop.speed_linear + 0.05)
                    action = f"เพิ่ม Linear Speed -> {teleop.speed_linear:.2f} m/s"
                elif key == '-' or key == '_':
                    teleop.speed_linear = max(0.05, teleop.speed_linear - 0.05)
                    action = f"ลด Linear Speed -> {teleop.speed_linear:.2f} m/s"
                elif key == ']':
                    teleop.speed_angular = min(4.0, teleop.speed_angular + 0.1)
                    action = f"เพิ่ม Angular Speed -> {teleop.speed_angular:.2f} rad/s"
                elif key == '[':
                    teleop.speed_angular = max(0.1, teleop.speed_angular - 0.1)
                    action = f"ลด Angular Speed -> {teleop.speed_angular:.2f} rad/s"
                elif key_lower == 'm':
                    teleop.strafe_mode = not teleop.strafe_mode
                    mode_name = "[สไลด์ข้าง]" if teleop.strafe_mode else "[เลี้ยวซ้าย-ขวา]"
                    action = f"เปลี่ยนโหมด A/D เป็น: {mode_name}"
                elif key == '\x03': # CTRL+C
                    break
                else:
                    action = f"กด: {repr(key)}"
                
                teleop.publish_twist(linear_x, linear_y, angular_z)
                print(f"\r[{action:<30}] Speed: L={teleop.speed_linear:.2f} m/s, A={teleop.speed_angular:.2f} rad/s    ", end='', flush=True)

    except Exception as e:
        print(f"\nError: {e}")
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
        teleop.publish_twist(0.0, 0.0, 0.0)
        teleop.destroy_node()
        rclpy.shutdown()
        print("\n\nหยุดการทำงาน และคืนค่า Terminal เรียบร้อยครับ\n")

if __name__ == '__main__':
    main()
