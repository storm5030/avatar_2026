import math
import time

from dynamixel_sdk import PortHandler, PacketHandler

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState

# 받아올 데이터의 크기 관리
ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION = 4
ADDR_TORQUE_ENABLE = 64
LEN_TORQUE_ENABLE = 1

# 모터는 현재 값을 불러올 때 0 - 1023 값을 0 - 300 도 사이로 받아옴.
# 모터의 튀어나온 부분이 150도 512인 것으로 보아 여기에도 보정이 필요해보임
# https://emanual.robotis.com/docs/kr/dxl/ax/ax-12a/ 
# 위 링크의 4.19 Present Position 참고
def raw_angle_to_rad(raw_angle):
    return float(raw_angle - 1600) / 4096.0 * 2 * math.pi

class BridgeNode(Node):

    def __init__(self):
        super().__init__('bridge_node')
        # 포트 정보
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        # 통신 속도
        self.declare_parameter('serial_baud', 1000000)
        # 다이나믹셀 id 설정
        self.declare_parameter('dxl_id', 2)
        #         
        self.declare_parameter('hz', 50.0)

        port = self.get_parameter('serial_port').value
        baud = int(self.get_parameter('serial_baud').value)
        self.dxl_id = int(self.get_parameter('dxl_id').value)        
        hz = float(self.get_parameter('hz').value)

        self.pub = self.create_publisher(JointState, '/joint_states', 10)

        # 다이나믹셀 자체 sdk 속 포트 핸들러 패킷 핸들러 이용
        # https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/sample_code/python_read_write_protocol_1_0/
        # 위 사이트에 자세한 사항 있으니 참고 바람
        self.port_handler = PortHandler(port)
        self.packet_handler = PacketHandler(2.0)

        # 포트 및 보드레이트 확인
        if self.port_handler.openPort():
            print("Succeeded to open the port!")
        else:
            raise RuntimeError(f'Falied to open port: {port}')
        
        if self.port_handler.setBaudRate(baud):
            print("Succeeded to change the baudrate!")
        else:
            raise RuntimeError(f'Falied to set baudrate: {baud}')
        
        # 토크 꺼졌나 확인
        if not self.torque_off():
            raise RuntimeError("Failed to turn torque off")
        
        self.dt = 1.0 / hz
        self.timer = self.create_timer(self.dt, self.tick)

    def tick(self):
        # https://emanual.robotis.com/docs/en/software/dynamixel/dynamixel_sdk/api_reference/python/python_packethandler/
        raw_angle, comm_result, error = self.packet_handler.read4ByteTxRx(self.port_handler, self.dxl_id, ADDR_PRESENT_POSITION)
        
        # 안정성 관련 부분
        if comm_result != 0 or error!= 0:
            self.get_logger().warn(f"DXL read failed: comm = {comm_result}")
            return
         
        rad_angle = raw_angle_to_rad(int(raw_angle))

        # 잘 프린팅 되고 있나 확인용으로 추가
        self.get_logger().info(
            f"[DXL] raw={raw_angle}, rad={rad_angle:.4f}",
            throttle_duration_sec=1.0
        )
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ["joint1"]
        msg.position = [float(rad_angle)]
        self.pub.publish(msg)

    # 토크 자동으로 꺼주는 함수 코드
    def torque_off(self):
        comm_result, error = self.packet_handler.write1ByteTxRx(
            self.port_handler, self.dxl_id, ADDR_TORQUE_ENABLE, 0
            )
        if comm_result != 0 or error != 0:
            self.get_logger().warn(f"Torque off failed: comm={comm_result}, error={error}")
            return False
        
        val, comm_result, error = self.packet_handler.read1ByteTxRx(
            self.port_handler, self.dxl_id, ADDR_TORQUE_ENABLE
        )
        self.get_logger().info(f"Torque Enable = {val}")
        return (comm_result == 0 and error == 0 and val == 0)

# 메인은 그냥 늘 하는것
def main():
    rclpy.init()
    node = BridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()