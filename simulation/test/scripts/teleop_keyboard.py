#!/usr/bin/env python3
import sys
import select
import termios
import tty
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

BANNER = """
====================================================================
 🎮 Mecanum 8-Direction Omnidirectional Keyboard Control
====================================================================
 [การเคลื่อนที่ 8 ทิศทาง / 8-Direction Movement (WASD + QEZC)]:
 
      Q (↖)       W (↑)       E (↗)
     (หน้า-ซ้าย)  (เดินหน้า)   (หน้า-ขวา)

      A (←)       S (↓)       D (→)
    (สไลด์ซ้าย)   (ถอยหลัง)   (สไลด์ขวา)

      Z (↙)                   C (↘)
    (หลัง-ซ้าย)               (หลัง-ขวา)

 ------------------------------------------------------------------
 [รองรับ Numpad 8 ทิศทาง]:
      7 (↖)   8 (↑)   9 (↗)
      4 (←)   5 (■)   6 (→)
      1 (↙)   2 (↓)   3 (↘)

 ------------------------------------------------------------------
 [การหมุนตัวรอบตัวเอง / In-Place Rotation]:
   J หรือ U : หมุนทวนเข็มนาฬิกา (Rotate Left  ↺)
   L หรือ O : หมุนตามเข็มนาฬิกา (Rotate Right ↻)

 ------------------------------------------------------------------
 [การปรับความเร็ว & ปุ่มหยุด]:
   + / = : เพิ่มความเร็วเชิงเส้น (+0.05 m/s)
   - / _ : ลดความเร็วเชิงเส้น (-0.05 m/s)
   ]     : เพิ่มความเร็วการหมุน (+0.1 rad/s)
   [     : ลดความเร็วการหมุน (-0.1 rad/s)
   SPACE หรือ X หรือ 5 : สั่งหยุดหุ่นยนต์ทันที (STOP)
   CTRL + C : ออกจากโปรแกรม (Quit)
====================================================================
"""

class MecanumTeleop(Node):
    def __init__(self):
        super().__init__('mecanum_teleop_keyboard')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        
        self.speed_linear = 0.35   # m/s
        self.speed_angular = 1.0   # rad/s

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
        # Check for arrow keys escape sequence
        if key == '\x1b':
            extra = sys.stdin.read(2)
            if extra == '[A':
                key = 'w' # Up arrow -> Forward
            elif extra == '[B':
                key = 's' # Down arrow -> Backward
            elif extra == '[C':
                key = 'd' # Right arrow -> Strafe right
            elif extra == '[D':
                key = 'a' # Left arrow -> Strafe left
    else:
        key = ''
    termios.tcsetattr(sys.stdin, termios.TCSADRAIN, settings)
    return key

def main(args=None):
    rclpy.init(args=args)
    teleop = MecanumTeleop()
    
    settings = termios.tcgetattr(sys.stdin)
    print(BANNER)
    
    try:
        linear_x = 0.0
        linear_y = 0.0
        angular_z = 0.0
        
        diag_factor = 1.0 / math.sqrt(2.0)
        
        while rclpy.ok():
            key = get_key(settings, timeout=0.1)
            
            if key:
                key_lower = key.lower()
                
                # 1. 4 ทิศทางตรง (Orthogonal Directions)
                if key_lower in ['w', '8']:
                    linear_x = teleop.speed_linear
                    linear_y = 0.0
                    angular_z = 0.0
                    action = "เดินหน้า ↑ (Forward)"
                elif key_lower in ['s', '2']:
                    linear_x = -teleop.speed_linear
                    linear_y = 0.0
                    angular_z = 0.0
                    action = "ถอยหลัง ↓ (Backward)"
                elif key_lower in ['a', '4']:
                    linear_x = 0.0
                    linear_y = teleop.speed_linear
                    angular_z = 0.0
                    action = "สไลด์ซ้าย ← (Strafe Left)"
                elif key_lower in ['d', '6']:
                    linear_x = 0.0
                    linear_y = -teleop.speed_linear
                    angular_z = 0.0
                    action = "สไลด์ขวา → (Strafe Right)"
                
                # 2. 4 ทิศทางเฉียง (Diagonal Directions)
                elif key_lower in ['q', '7']:
                    linear_x = teleop.speed_linear * diag_factor
                    linear_y = teleop.speed_linear * diag_factor
                    angular_z = 0.0
                    action = "เฉียงหน้า-ซ้าย ↖ (Forward-Left)"
                elif key_lower in ['e', '9']:
                    linear_x = teleop.speed_linear * diag_factor
                    linear_y = -teleop.speed_linear * diag_factor
                    angular_z = 0.0
                    action = "เฉียงหน้า-ขวา ↗ (Forward-Right)"
                elif key_lower in ['z', '1']:
                    linear_x = -teleop.speed_linear * diag_factor
                    linear_y = teleop.speed_linear * diag_factor
                    angular_z = 0.0
                    action = "เฉียงหลัง-ซ้าย ↙ (Backward-Left)"
                elif key_lower in ['c', '3']:
                    linear_x = -teleop.speed_linear * diag_factor
                    linear_y = -teleop.speed_linear * diag_factor
                    angular_z = 0.0
                    action = "เฉียงหลัง-ขวา ↘ (Backward-Right)"
                
                # 3. การหมุนตัวรอบตัวเอง (In-Place Rotation)
                elif key_lower in ['j', 'u']:
                    linear_x = 0.0
                    linear_y = 0.0
                    angular_z = teleop.speed_angular
                    action = "หมุนซ้าย ↺ (Rotate Left)"
                elif key_lower in ['l', 'o']:
                    linear_x = 0.0
                    linear_y = 0.0
                    angular_z = -teleop.speed_angular
                    action = "หมุนขวา ↻ (Rotate Right)"
                
                # 4. หยุดฉุกเฉิน (Emergency Stop)
                elif key_lower in [' ', 'x', '5', 'k']:
                    linear_x = 0.0
                    linear_y = 0.0
                    angular_z = 0.0
                    action = "หยุดสนิท ■ (STOP)"
                
                # 5. ปรับความเร็ว
                elif key in ['+', '=']:
                    teleop.speed_linear = min(3.0, teleop.speed_linear + 0.05)
                    action = f"เพิ่ม Linear Speed -> {teleop.speed_linear:.2f} m/s"
                elif key in ['-', '_']:
                    teleop.speed_linear = max(0.05, teleop.speed_linear - 0.05)
                    action = f"ลด Linear Speed -> {teleop.speed_linear:.2f} m/s"
                elif key == ']':
                    teleop.speed_angular = min(5.0, teleop.speed_angular + 0.1)
                    action = f"เพิ่ม Angular Speed -> {teleop.speed_angular:.2f} rad/s"
                elif key == '[':
                    teleop.speed_angular = max(0.1, teleop.speed_angular - 0.1)
                    action = f"ลด Angular Speed -> {teleop.speed_angular:.2f} rad/s"
                elif key == '\x03': # CTRL+C
                    break
                else:
                    action = f"กดปุ่ม: {repr(key)}"
                
                teleop.publish_twist(linear_x, linear_y, angular_z)
                print(f"\r[{action:<35}] Linear: {teleop.speed_linear:.2f} m/s | Angular: {teleop.speed_angular:.2f} rad/s    ", end='', flush=True)

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
