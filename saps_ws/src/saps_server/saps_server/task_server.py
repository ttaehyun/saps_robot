import time
import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# saps_interfaces에서 빌드한 RobotTask 액션을 임포트
from saps_interfaces.action import RobotTask

class LocalTaskPlanner(Node):
    def __init__(self):
        super().__init__('local_task_planner')
        
        # 내부에서 다른 액션/서비스 호출 시 교착 상태를 방지하기 위한 콜백 그룹
        self.callback_group = ReentrantCallbackGroup()
        
        self._action_server = ActionServer(
            self,
            RobotTask,
            'robot_task',              # Action 통신을 위한 토픽(이름)
            execute_callback=self.execute_callback,
            callback_group=self.callback_group
        )
        self.get_logger().info('Local Task Planner (Director Node) started. Waiting for goals...')

    async def execute_callback(self, goal_handle):
        task_name = goal_handle.request.task_name
        self.get_logger().info(f'--- Mission Started: {task_name} ---')
        
        feedback_msg = RobotTask.Feedback()
        result = RobotTask.Result()

        # 요청된 Task 종류에 따라 분기 처리 (State Machine / Sequence)
        if task_name == 'patrol':
            success = await self.execute_patrol_sequence(goal_handle, feedback_msg)
        elif task_name == 'delivery':
            success = await self.execute_delivery_sequence(goal_handle, feedback_msg)
        else:
            self.get_logger().warn(f'Unknown task requested: {task_name}')
            success = False
            
        # 최종 결과 처리
        if success:
            goal_handle.succeed()
            result.success = True
            self.get_logger().info(f'--- Mission [{task_name}] Succeeded! ---')
        else:
            goal_handle.abort()
            result.success = False
            self.get_logger().error(f'--- Mission [{task_name}] Failed or Aborted! ---')
        
        return result

    async def execute_patrol_sequence(self, goal_handle, feedback_msg):
        # [시퀀스 1] A 지점 이동 (추후 Nav2 Action Client 호출 로직으로 대체)
        feedback_msg.current_status = 'Moving to Point A (Nav2...)'
        feedback_msg.percent_complete = 33.3
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info(feedback_msg.current_status)
        time.sleep(2.0) # 실제로는 Nav2 이동 완료까지 await로 대기
        
        # [시퀀스 2] 주변 스캔 및 사진 촬영 등 특정 액션
        feedback_msg.current_status = 'Scanning area...'
        feedback_msg.percent_complete = 66.6
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info(feedback_msg.current_status)
        time.sleep(2.0)

        feedback_msg.current_status = 'Patrol Complete. Returning to standby.'
        feedback_msg.percent_complete = 100.0
        goal_handle.publish_feedback(feedback_msg)
        return True

    async def execute_delivery_sequence(self, goal_handle, feedback_msg):
        # 배달 시퀀스 로직 예시
        self.get_logger().info('Executing delivery logic...')
        time.sleep(3.0)
        return True

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
