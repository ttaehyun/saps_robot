import time
import rclpy
from rclpy.action import ActionServer, ActionClient, CancelResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# saps_interfaces에서 빌드한 RobotTask 액션을 임포트
from saps_interfaces.action import RoverCommand
from geometry_msgs.msg import PoseStamped
class LocalTaskPlanner(Node):
    def __init__(self):
        super().__init__('local_task_planner')
        
        # 내부에서 다른 액션/서비스 호출 시 교착 상태를 방지하기 위한 콜백 그룹
        self.callback_group = ReentrantCallbackGroup()
    
        self._action_server = ActionServer(
            self,
            RoverCommand,
            '/rover_command',              # Action 통신을 위한 토픽(이름)
            execute_callback=self.execute_callback,
            cancel_callback=self.cancel_callback,
            callback_group=self.callback_group
        )

        self.goal_pose_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)

        self.get_logger().info('Local Task Planner (Director Node) started. Waiting for goals...')
        
    def cancel_callback(self, goal_handle):
        self.get_logger().info('Client requested to cancel the goal!')
        # 취소 요청을 수락(ACCEPT)합니다.
        return CancelResponse.ACCEPT

    async def execute_callback(self, goal_handle):
        command = goal_handle.request.command
        
        self.get_logger().info(f'--- Mission Started: {command} ')
        
        feedback_msg = RoverCommand.Feedback()
        result = RoverCommand.Result()

        # 요청된 Task 종류에 따라 분기 처리 (State Machine / Sequence)
        if command == 'start':
            status = await self.execute_patrol_sequence(goal_handle, feedback_msg)
        elif command == 'return':
            status = await self.execute_delivery_sequence(goal_handle, feedback_msg)
        else:
            self.get_logger().warn(f'Unknown task requested: {command}')
            status = 'FAILED'
            
        # 최종 결과 처리
        if status == 'SUCCESS':
            goal_handle.succeed()
            result.success = True
            self.get_logger().info(f'--- Mission [{command}] Succeeded! ---')
        elif status == 'CANCELED':
            goal_handle.canceled()
            result.success = False
            self.get_logger().warn(f'--- Mission [{command}] Canceled! ---')
        else:
            goal_handle.abort()
            result.success = False
            self.get_logger().error(f'--- Mission [{command}] Failed or Aborted! ---')
        
        return result

    async def execute_patrol_sequence(self, goal_handle, feedback_msg):
        # [시퀀스 1] A 지점 이동 (추후 Nav2 Action Client 호출 로직으로 대체)
        feedback_msg.status = 'Moving to Point A (Nav2...)'
        feedback_msg.progress = 33.3
        target_x = goal_handle.request.x
        target_y = goal_handle.request.y
        self.get_logger().error(f'(Target: x={target_x}, y={target_y}) ---')
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info(feedback_msg.status)
        
        # time.sleep() 대신 짧게 대기하며 취소 요청(is_cancel_requested)을 주기적으로 확인합니다.
        for _ in range(20):
            if goal_handle.is_cancel_requested:
                feedback_msg.status = 'Mission Canceled by Client (during Move to Point A).'
                goal_handle.publish_feedback(feedback_msg)
                self.get_logger().info(feedback_msg.status)
                # 취소 요청 시 모터 정지 등 필요한 정리 작업을 여기에 추가하세요.
                return 'CANCELED'
            time.sleep(0.1) # 실제로는 Nav2 이동 중 상태를 확인하며 대기
        
        # [시퀀스 2] 주변 스캔 및 사진 촬영 등 특정 액션
        feedback_msg.status = 'Scanning area...'
        feedback_msg.progress = 66.6
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info(feedback_msg.status)
        
        for _ in range(50):
            if goal_handle.is_cancel_requested:
                feedback_msg.status = 'Mission Canceled by Client (during Scanning).'
                goal_handle.publish_feedback(feedback_msg)
                self.get_logger().info(feedback_msg.status)
                return 'CANCELED'
            time.sleep(0.1)

        feedback_msg.status = 'Patrol Complete. Returning to standby.'
        feedback_msg.progress = 100.0
        goal_handle.publish_feedback(feedback_msg)
        return 'SUCCESS'

    async def execute_delivery_sequence(self, goal_handle, feedback_msg):
        # 배달 시퀀스 로직 예시
        self.get_logger().info('Executing delivery logic...')
        
        for _ in range(30):
            if goal_handle.is_cancel_requested:
                feedback_msg.status = 'Delivery Mission Canceled by Client.'
                goal_handle.publish_feedback(feedback_msg)
                self.get_logger().info(feedback_msg.status)
                return 'CANCELED'
            time.sleep(0.1)
            
        return 'SUCCESS'

def main(args=None):
    rclpy.init(args=args)
    node = LocalTaskPlanner()
    
    # Action Server 내부에서 다른 Action Client를 사용하기 위해 MultiThreadedExecutor 사용
    executor = MultiThreadedExecutor()
    try:
        rclpy.spin(node, executor=executor)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down Local Task Planner...')
    finally:
        node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
