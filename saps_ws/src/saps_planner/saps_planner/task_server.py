#!/usr/bin/env python3
import time
import rclpy
from rclpy.action import ActionServer, ActionClient, CancelResponse
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

# saps_interfaces에서 빌드한 RobotTask 액션을 임포트
from saps_interfaces.action import RoverCommand
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

# ros2 action send_goal /rover_command saps_interfaces/action/RoverCommand "{command: 'start', x: 1.48, y: 0.67}" -f

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

        # Nav2 목표 지점 이동을 위한 Action Client 구성
        self.nav_to_pose_client = ActionClient(
            self,
            NavigateToPose,
            'navigate_to_pose',
            callback_group=self.callback_group
        )

        # 돌 정렬(Align)을 위한 Action Client 구성
        self.align_stone_client = ActionClient(
            self,
            RoverCommand,
            '/align_stone',
            callback_group=self.callback_group
        )

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
        target_x = goal_handle.request.x
        target_y = goal_handle.request.y

        # [시퀀스 1] A 지점 이동 (Nav2 Action Client 호출)
        feedback_msg.status = f'Moving to Point A (x={target_x}, y={target_y}) via Nav2...'
        feedback_msg.progress = 10.0
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info(feedback_msg.status)
        
        # Nav2 서버 연결 대기
        if not self.nav_to_pose_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Nav2 navigate_to_pose action server not available!')
            return 'FAILED'

        # Nav2 목표 설정
        nav_goal = NavigateToPose.Goal()
        nav_goal.pose.header.frame_id = 'map'
        nav_goal.pose.header.stamp = self.get_clock().now().to_msg()
        nav_goal.pose.pose.position.x = float(target_x)
        nav_goal.pose.pose.position.y = float(target_y)
        nav_goal.pose.pose.orientation.w = 1.0  # 기본 방향(회전 없음) 설정

        # 비동기로 목표 전송
        send_goal_future = self.nav_to_pose_client.send_goal_async(nav_goal)
        while rclpy.ok() and not send_goal_future.done():
            if goal_handle.is_cancel_requested:
                send_goal_future.cancel()
                return 'CANCELED'
            time.sleep(0.1)
            
        nav_goal_handle = send_goal_future.result()
        if not nav_goal_handle.accepted:
            self.get_logger().error('Nav2 goal was rejected by server')
            return 'FAILED'
            
        # Nav2 동작 완료 대기 및 지속적인 취소 상태 체크
        result_future = nav_goal_handle.get_result_async()
        while rclpy.ok() and not result_future.done():
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Cancel requested, forwarding cancellation to Nav2...')
                cancel_future = nav_goal_handle.cancel_goal_async()
                while rclpy.ok() and not cancel_future.done():
                    time.sleep(0.1)
                self.get_logger().info('Nav2 goal successfully canceled.')
                return 'CANCELED'
            time.sleep(0.1)

        # 결과 확인
        nav_result = result_future.result()
        if nav_result.status != GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().error(f'Navigation failed with status: {nav_result.status}')
            return 'FAILED'
        
        self.get_logger().info('Navigation to Target Succeeded!')
        
        # [시퀀스 2] 돌과 차량 정렬 액션 호출
        feedback_msg.status = 'Aligning to stone...'
        feedback_msg.progress = 66.6
        goal_handle.publish_feedback(feedback_msg)
        self.get_logger().info(feedback_msg.status)
        
        if not self.align_stone_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error('Align stone action server not available!')
            return 'FAILED'

        align_goal = RoverCommand.Goal()
        align_goal.command = 'align'

        align_goal_future = self.align_stone_client.send_goal_async(align_goal)
        while rclpy.ok() and not align_goal_future.done():
            if goal_handle.is_cancel_requested:
                align_goal_future.cancel()
                return 'CANCELED'
            time.sleep(0.1)
            
        align_goal_handle = align_goal_future.result()
        if not align_goal_handle.accepted:
            self.get_logger().error('Align stone goal was rejected by server')
            return 'FAILED'
            
        align_result_future = align_goal_handle.get_result_async()
        while rclpy.ok() and not align_result_future.done():
            if goal_handle.is_cancel_requested:
                self.get_logger().info('Cancel requested, forwarding cancellation to Align server...')
                cancel_future = align_goal_handle.cancel_goal_async()
                while rclpy.ok() and not cancel_future.done():
                    time.sleep(0.1)
                self.get_logger().info('Align goal successfully canceled.')
                return 'CANCELED'
            time.sleep(0.1)

        align_result = align_result_future.result().result
        if not align_result.success:
            self.get_logger().error(f'Alignment failed: {align_result.message}')
            return 'FAILED'
        
        self.get_logger().info('Alignment to Stone Succeeded!')

        feedback_msg.status = 'Mission Complete. Stone Aligned.'
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
