import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer

from saps_interfaces.action import RoverCommand


# =========================================================
# 실제 로버 제어 함수
# 지금은 print만 있음
# 나중에 여기에 실제 모터/집게 코드를 넣으면 됨
# =========================================================

def motor_stop():
    print("[ROVER] 모터 정지")
    # TODO: 실제 정지 코드 넣기


def move_to_target(x, y):
    print(f"[ROVER] 돌 좌표로 이동 시작: x={x}, y={y}")
    # TODO: 실제 좌표 이동 코드 넣기
    time.sleep(2.0)
    print("[ROVER] 돌 좌표 도착")


def gripper_close():
    print("[ROVER] 집게 닫기")
    # TODO: 실제 집게 닫기 코드 넣기
    time.sleep(1.0)


def return_to_home():
    print("[ROVER] 원래 위치로 복귀 시작")
    # TODO: 실제 복귀 코드 넣기
    time.sleep(2.0)
    print("[ROVER] 원래 위치 복귀 완료")


class RoverActionServer(Node):
    def __init__(self):
        super().__init__('rover_action_server')

        self.action_server = ActionServer(
            self,
            RoverCommand,
            '/rover_command',
            self.execute_callback
        )

        self.get_logger().info("로버 Action Server 실행 중...")
        self.get_logger().info("Action name: /rover_command")

    def send_feedback(self, goal_handle, status, progress):
        feedback = RoverCommand.Feedback()
        feedback.status = status
        feedback.progress = float(progress)
        goal_handle.publish_feedback(feedback)

        self.get_logger().info(
            f"Feedback 전송: {status}, progress={progress}%"
        )

    def execute_callback(self, goal_handle):
        command = goal_handle.request.command
        x = goal_handle.request.x
        y = goal_handle.request.y

        self.get_logger().info(
            f"Goal 수신: command={command}, x={x}, y={y}"
        )

        result = RoverCommand.Result()

        try:
            if command == "start":
                self.send_feedback(goal_handle, "start_received", 0.0)

                self.get_logger().info(
                    f"출발 명령 실행: 돌 좌표 x={x}, y={y}"
                )

                self.send_feedback(goal_handle, "moving_to_target", 30.0)
                move_to_target(x, y)

                self.send_feedback(goal_handle, "arrived_target", 70.0)

                self.send_feedback(goal_handle, "grabbing_object", 85.0)
                gripper_close()

                self.send_feedback(goal_handle, "mission_complete", 100.0)

                result.success = True
                result.message = f"start 완료: x={x}, y={y} 이동 후 집기 완료"
                goal_handle.succeed()
                return result

            elif command == "return":
                self.send_feedback(goal_handle, "return_received", 0.0)

                self.get_logger().info("복귀 명령 실행")

                self.send_feedback(goal_handle, "returning_home", 50.0)
                return_to_home()

                self.send_feedback(goal_handle, "return_complete", 100.0)

                result.success = True
                result.message = "return 완료: 원래 위치 복귀 완료"
                goal_handle.succeed()
                return result

            elif command == "emergency_stop":
                self.send_feedback(goal_handle, "emergency_stop_received", 0.0)

                self.get_logger().warn("긴급정지 실행")
                motor_stop()

                self.send_feedback(goal_handle, "emergency_stopped", 100.0)

                result.success = True
                result.message = "emergency_stop 완료"
                goal_handle.succeed()
                return result

            else:
                self.get_logger().warn(f"알 수 없는 명령: {command}")
                result.success = False
                result.message = f"알 수 없는 명령: {command}"
                goal_handle.abort()
                return result

        except Exception as e:
            self.get_logger().error(f"Action 실행 중 오류: {e}")
            motor_stop()

            result.success = False
            result.message = f"오류 발생: {str(e)}"
            goal_handle.abort()
            return result


def main(args=None):
    rclpy.init(args=args)

    node = RoverActionServer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()