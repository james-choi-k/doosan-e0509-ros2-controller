import rclpy
from rclpy.node import Node
from dsr_msgs2.srv import MoveJoint
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math

class SimpleMoveServer(Node):
    def __init__(self):
        super().__init__('simple_move_server')
        # 서비스 서버 (GUI로부터 명령 받음)
        self.srv = self.create_service(MoveJoint, 'motion/move_joint', self.move_callback)
        # 가제보 컨트롤러에게 직접 명령 (토픽)
        self.pub = self.create_publisher(JointTrajectory, '/gz/dsr_joint_trajectory_controller/joint_trajectory', 10)        
        self.get_logger().info('최종 이동 서버가 준비되었습니다!')

    def move_callback(self, request, response):
        self.get_logger().info(f'요청 받은 각도(Deg): {request.pos}')
        
        msg = JointTrajectory()
        # 가제보 모델의 실제 joint 이름과 일치해야 합니다.
        msg.joint_names = ['joint_1', 'joint_2', 'joint_3', 'joint_4', 'joint_5', 'joint_6']
        
        point = JointTrajectoryPoint()
        # 중요: 가제보는 Radian을 사용하므로 Degree를 Radian으로 변환합니다.
        point.positions = [math.radians(p) for p in request.pos]
        point.time_from_start.sec = 1  # 1초 동안 이동
        point.time_from_start.nanosec = 0
        
        msg.points.append(point)
        self.pub.publish(msg)
        
        response.success = True
        return response

def main():
    rclpy.init()
    node = SimpleMoveServer()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()