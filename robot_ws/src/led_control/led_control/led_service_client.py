import rclpy
from rclpy.node import Node
from std_srvs.srv import SetBool

class LedClient(Node):
    def __init__(self):
        super().__init__('led_client')
        self.client = self.create_client(SetBool, 'toggle_led')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')
        self.turn_led_on()

    def turn_led_on(self):
        request = SetBool.Request()
        request.data = True  # LED를 켜기 위해 True로 설정
        self.future = self.client.call_async(request)
        self.future.add_done_callback(self.response_callback)

    def response_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info(f"LED state: {response.message}")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = LedClient()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()
