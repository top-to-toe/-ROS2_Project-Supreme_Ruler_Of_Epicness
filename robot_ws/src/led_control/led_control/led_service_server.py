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
        str_data = str(request.data).lower()
        self.get_logger().info(f'Received request: {str_data} (converted from: {request.data}, type: {type(request.data)})')

        led_state = str_data not in ['false', '0']

        g.output(21, led_state)  # LED 켜기/끄기
        response.success = True
        response.message = "LED is ON" if led_state else "LED is OFF"
        self.get_logger().info(f'LED should now be {"ON" if led_state else "OFF"}')

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
