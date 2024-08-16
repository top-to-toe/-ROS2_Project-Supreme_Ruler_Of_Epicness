import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool
import RPi.GPIO as g

# GPIO 초기 설정
g.setmode(g.BCM)
g.setup(21, g.OUT)

class Hbmove(Node):
    def __init__(self):
        super().__init__("led_service_server")
        self.srv = self.create_service(SetBool, 'toggle_led', self.toggle_led_callback)

    def toggle_led_callback(self, request, response):
        # 모든 입력을 문자열로 변환 후 처리
        str_data = str(request.data).lower()
        self.get_logger().info(f'Received request: {str_data} (converted from: {request.data}, type: {type(request.data)})')

        # 정확히 'false' 또는 '0'인 경우 False로 처리
        if str_data == 'false' or str_data == '0':
            led_state = False
        else:
            led_state = True

        # LED 상태 설정
        if led_state:
            g.output(21, True)  # LED 켬
            response.success = True
            response.message = "LED is ON"
            self.get_logger().info('LED should now be ON')
        else:
            g.output(21, False)  # LED 끔
            response.success = True
            response.message = "LED is OFF"
            self.get_logger().info('LED should now be OFF')

        return response

def main():
    rclpy.init()
    node = Hbmove()
    try:
        rclpy.spin(node)
    except Exception as e:
        print(f"Exception: {e}")
    finally:
        g.cleanup(21)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == "__main__":
    main()
