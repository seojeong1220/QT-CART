import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator
import socket

UDP_PORT_QT = 55555   # Qt 포트
UDP_PORT_UWB = 44444  # UWB 포트
BUFFER_SIZE = 1024

class RosController(Node):
    def __init__(self):
        super().__init__('ros_controller_node')
        
        # ROS 2 퍼블리셔 & Nav2
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.navigator = BasicNavigator()
        
        # UDP 소켓 설정
        self.sock_qt = self.create_udp_socket(UDP_PORT_QT)
        self.sock_uwb = self.create_udp_socket(UDP_PORT_UWB)

        # 상태 변수
        self.current_mode = 0 # 0:정지, 1:따라가기, 2:자율주행
        self.uwb_L = 0.0
        self.uwb_R = 0.0

        # 20Hz 주기로 제어 루프 실행
        self.create_timer(0.05, self.control_loop)
        self.get_logger().info(f"ROS Controller Started! (UDP: {UDP_PORT_QT}, {UDP_PORT_UWB})")

    def create_udp_socket(self, port):
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.bind(('0.0.0.0', port))
        sock.setblocking(False) 
        return sock

    def control_loop(self):
        self.receive_uwb_data()
        self.receive_qt_command()
        
        msg = Twist()
        
        if self.current_mode == 0:     # 정지
            self.cmd_vel_pub.publish(msg)
            
        elif self.current_mode == 1:   # 따라가기
            self.process_follow_mode(msg)
            
        elif self.current_mode == 2:   # 자율주행 (Nav2)
            if self.navigator.isTaskComplete():
                pass # 도착했거나 작업 끝남

    def receive_uwb_data(self):
        try:
            while True: # 버퍼 비우기
                data, _ = self.sock_uwb.recvfrom(BUFFER_SIZE)
                text = data.decode('utf-8').strip()
                if text.startswith("L:"): self.uwb_L = float(text[2:])
                elif text.startswith("R:"): self.uwb_R = float(text[2:])
        except BlockingIOError: pass
        except ValueError: pass

    def receive_qt_command(self):
        try:
            while True:
                data, _ = self.sock_qt.recvfrom(BUFFER_SIZE)
                text = data.decode('utf-8').strip()
                
                if text.startswith("MODE:"):
                    self.change_mode(int(text.split(":")[1]))
                elif text.startswith("GOAL:"):
                    parts = text.split(":")[1].split(",")
                    self.start_navigation(float(parts[0]), float(parts[1]))
        except BlockingIOError: pass
        except Exception as e: self.get_logger().error(f"Qt Cmd Error: {e}")

    def change_mode(self, new_mode):
        if self.current_mode == new_mode: return
        
        self.get_logger().info(f"Mode: {self.current_mode} -> {new_mode}")
        
        # 자율주행 하다가 다른 모드로 가면 Nav2 취소
        if self.current_mode == 2 and new_mode != 2:
            self.get_logger().warn("Cancelling Navigation Task...")
            self.navigator.cancelTask()
            
        self.current_mode = new_mode

    def process_follow_mode(self, msg):
        l, r = self.uwb_L, self.uwb_R
        
        if l <= 0.01 or r <= 0.01:
            self.cmd_vel_pub.publish(msg)
            return
        
        avg = (l + r) / 2.0
        diff = r - l
        
        if abs(diff) > 0.1: 
            msg.angular.z = -(diff * 2.0) 

        if avg > 1.2: 
            msg.linear.x = -0.2  
        elif avg < 0.8:          
            msg.linear.x = 0.0
        
        self.cmd_vel_pub.publish(msg)
    def start_navigation(self, x, y):
        # 강제로 모드 2 전환
        if self.current_mode != 2: self.change_mode(2)
        
        self.get_logger().info(f"UWB x={x}, y={y}")
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.navigator.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.w = 1.0
        self.navigator.goToPose(goal)

def main():
    rclpy.init()
    node = RosController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()