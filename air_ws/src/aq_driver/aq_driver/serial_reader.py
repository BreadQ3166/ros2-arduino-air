import rclpy
from rclpy.node import Node
from aq_msgs.msg import AirQuality
import serial

class AirQualitySerialNode(Node):
    def __init__(self):
        super().__init__('air_quality_serial_node')

        # 1. 声明参数
        self.declare_parameter('port_name', '/dev/ttyACM0')
        self.declare_parameter('baud_rate', 115200)

        # 2. 创建发布者
        self.publisher_ = self.create_publisher(AirQuality, 'air_quality_data', 10)

        # 3. 初始化变量
        self.ser = None
        self.timer = self.create_timer(0.1, self.timer_callback) # 10Hz

        self.get_logger().info('Air Quality Node Started. Waiting for loop...')

    def open_serial(self):
        port = self.get_parameter('port_name').value
        baud = self.get_parameter('baud_rate').value
        try:
            if self.ser is None or not self.ser.is_open:
                self.ser = serial.Serial(port, baud, timeout=0.1)
                self.get_logger().info(f'Successfully opened serial port: {port}')
                return True
        except Exception as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            return False
        return True

    def close_serial(self):
        if self.ser and self.ser.is_open:
            self.ser.close()
            self.get_logger().info('Serial port closed.')

    def timer_callback(self):
        
        msg = AirQuality()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_Link"

        # === 真实串口模式 ===
        if not (self.ser and self.ser.is_open):
            if not self.open_serial():
                return # 打开失败，跳过本次循环

        try:
            # 读取一行数据: $START,12.5,120.5,$END
            if self.ser.in_waiting > 0:
                line = self.ser.readline().decode('utf-8').strip()
                parts = line.split(',')
                
                # 简单校验
                if len(parts) == 7 and parts[0] == '$START' and parts[6] == '$END':
                    msg.air_quality = int(parts[1])
                    msg.dust = float(parts[2])
                    msg.tvoc = float(parts[3])
                    msg.co2 = int(parts[4])
                    msg.ch2o = int(parts[5])

                    self.publisher_.publish(msg)
                    
                else:
                    self.get_logger().warn(f'Invalid data format: {line}')
        except Exception as e:
            self.get_logger().error(f'Serial read error: {e}')
            self.close_serial() # 出错则关闭，尝试重连

def main(args=None):
    rclpy.init(args=args)
    node = AirQualitySerialNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_serial()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()