import rclpy  #ros2的python接口
from rclpy.node import Node  # 导入Node类,用于创建节点
from referee_msg.msg import Referee # 导入自定义消息类型，这个是自己写的裁判系统消息类型
from geometry_msgs.msg import Twist # 导入Twist消息类型，用于控制机器人运动
import serial  # 导入串口模块
import json  # 导入json模块
import struct # 导入struct模块,用于打包数据
import threading  # 导入线程模块
from std_msgs.msg import Int8  # 状态消息类型
class SerialNode(Node):
    def __init__(self):
        super().__init__('serial_node')

        # 设置串口参数
        self.serial_port = '/dev/ttyUSB0'  # 使用实际存在的串口路径
        self.baud_rate = 115200
        self.get_logger().info(f'Serial port set to: {self.serial_port}')
        self.Status_nav2 = 0
        # 初始化串口
        self.serial_conn = None
        try:
            self.serial_conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
            self.get_logger().info(f'Connected to {self.serial_port} at {self.baud_rate} baud rate.')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to connect to {self.serial_port}: {e}')
            # 错误处理
            self.destroy_node()
            rclpy.shutdown()
        # 创建订阅者，订阅导航数据话题，把计算好的数据发给单片机
        self.subscription = self.create_subscription(Twist, '/cmd_vel', self.SendtoSTM32_callback, 10)
        self.subscription_1 = self.create_subscription(Int8, 'nav2_status',self.Nav2Stat_callback,10)
        # 创建发布者,将接受到的来自单片机的数据发布到/stm32_ros2_data话题
        self.publisher_ = self.create_publisher(Referee, 'stm32_ros2_data', 10)

        # 创建定时器，定期读取串口数据
        self.timer = self.create_timer(0.1, self.read_serial_data)

    def read_serial_data(self):
        if self.serial_conn and self.serial_conn.is_open:
            try:
                data = self.serial_conn.readline().decode('utf-8',errors='ignore').strip()
                if data:
                    try:
                        # 尝试解析JSON数据
                        parsed_data = json.loads(data)
                        self.process_data(parsed_data)
                    except (json.JSONDecodeError, ValueError, TypeError) as e:
                        self.get_logger().error(f'Failed to parse JSON: {e}')
                        return
            except serial.SerialException as e:
                self.get_logger().error(f'Error reading serial data: {e}')
        else:
            self.get_logger().warning('Serial connection is not open.')

    def process_data(self, data):
        # 处理解析后的数据，根据实际需求进行相应操作
        msg = Referee()
        msg.game_type = int(data.get('game_type'))#比赛类型
        msg.game_progress = int(data.get('game_progress'))#比赛阶段——4 比赛进行中
        msg.remain_hp = int(data.get('remain_hp'))#机器人当前血量
        msg.max_hp = int(data.get('max_hp'))#。。。
        msg.stage_remain_time = int(data.get('stage_remain_time'))#当前阶段剩余时间，                     
        msg.bullet_remaining_num_17mm = int(data.get('bullet_remaining_num_17mm'))#剩余发弹量
        msg.red_outpost_hp = int(data.get('red_outpost_hp'))    
        msg.red_base_hp = int(data.get('red_base_hp'))
        msg.blue_outpost_hp = int(data.get('blue_outpost_hp'))
        msg.blue_base_hp = int(data.get('blue_base_hp'))
        msg.rfid_status = int(data.get('rfid_status'))#rfid状态
        # 发布消息
        self.publisher_.publish(msg)
    def Nav2Stat_callback(self,msg):
         self.Status_nav2 = msg.data
    def SendtoSTM32_callback(self, msg):
        # 接收来自ROS2的指令，并发送给单片机
        if self.serial_conn and self.serial_conn.is_open:
            try:
                # 数据字段定义
                header = 0xAA
                checksum = 19
                x_speed = -msg.linear.x *0.7
                y_speed = -msg.linear.y *0.7
                rotate = msg.angular.z 
                yaw_speed = msg.angular.z *1.5
                # yaw_speed = 10
                running_state = 0x00
                data_frame = struct.pack(
                    '<BBffffB',  # 格式化字符串：<表示小端，B表示uint8_t，f表示float
                    header,         # uint8_t
                    checksum,       # uint8_t
                    x_speed,        # float
                    y_speed,        # float
                    rotate,         # float
                    yaw_speed,      # float
                    running_state   # uint8_t
                )
                # 发送数据
                self.serial_conn.write(data_frame)
                self.get_logger().info('Sent data to STM32')
            except serial.SerialException as e:
                self.get_logger().error(f'Error sending data to STM32: {e}')
        else:
            self.get_logger().warning('Serial connection is not open.')

    def __del__(self):
        if self.serial_conn and self.serial_conn.is_open:
            self.serial_conn.close()
            self.get_logger().info(f'Serial connection to {self.serial_port} closed.')

def ros_spin_thread(node):
    rclpy.spin(node)

def main(args=None):
    rclpy.init(args=args)
    serial_node = SerialNode()
    spin_thread = threading.Thread(target=ros_spin_thread, args=(serial_node,))
    spin_thread.start()
    spin_thread.join()
    serial_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()






# import rclpy  # 导入rclpy库
# from rclpy.node import Node  # 导入Node类
# from referee_msg.msg import Referee  # 导入自定义的裁判系统消息
# from geometry_msgs.msg import Twist  # 导入Twist消息（在此文件中已不直接使用）
# import serial  # 导入串口通信库
# import json  # 导入JSON库
# import struct  # 导入struct库用于数据打包
# import threading  # 导入线程库（在此修改版中main函数已简化，不再需要）
# from std_msgs.msg import Int8  # 导入Int8消息（在此文件中已不直接使用）

# class SerialNode(Node):
#     def __init__(self):
#         """
#         节点初始化
#         """
#         super().__init__('serial_node_debug')

#         # 设置串口参数
#         self.serial_port = '/dev/ttyUSB0'  # 请确保使用实际存在的串口路径
#         self.baud_rate = 115200
#         self.get_logger().info(f'串口设置为: {self.serial_port}')

#         # 初始化串口
#         self.serial_conn = None
#         try:
#             self.serial_conn = serial.Serial(self.serial_port, self.baud_rate, timeout=1)
#             self.get_logger().info(f'成功连接到 {self.serial_port}，波特率 {self.baud_rate}。')
#         except serial.SerialException as e:
#             self.get_logger().error(f'无法连接到串口 {self.serial_port}: {e}')
#             # 如果串口连接失败，则销毁节点并关闭rclpy
#             self.destroy_node()
#             rclpy.shutdown()
#             return

#         # 【修改】: 注释掉订阅者，不再需要回调来触发发送
#         # self.subscription = self.create_subscription(Twist, '/cmd_vel', self.SendtoSTM32_callback, 10)
#         # self.subscription_1 = self.create_subscription(Int8, 'nav2_status',self.Nav2Stat_callback,10)

#         # 创建发布者，用于将从单片机收到的数据发布出去
#         self.publisher_ = self.create_publisher(Referee, 'stm32_ros2_data', 10)
        
#         # 【新增】: 创建一个定时器，周期性地直接发送调试数据
#         self.send_timer = self.create_timer(0.5, self.send_debug_data)  # 每0.5秒发送一次

#         # 创建定时器，周期性地直接读取串口数据
#         self.read_timer = self.create_timer(0.1, self.read_serial_data)

#     def send_debug_data(self):
#         """
#         直接发送调试数据到下位机，所有数据字段都置为1。
#         此函数由定时器周期性调用。
#         """
#         if self.serial_conn and self.serial_conn.is_open:
#             try:
#                 # 定义所有要发送的数据字段，并全部置1
#                 header = 0xAA          # 帧头 (uint8_t)
#                 checksum = 19       # 校验和 (uint8_t)
#                 x_speed = 1.0       # x轴速度 (float)
#                 y_speed = 1.0       # y轴速度 (float)
#                 rotate = 1.0        # 旋转 (float)
#                 yaw_speed = 1.0     # yaw轴速度 (float)
#                 running_state = 1   # 运行状态 (uint8_t)

#                 # 使用struct将数据打包成二进制格式
#                 # '<' 表示小端模式
#                 # 'B' 表示无符号字符 (1字节), 'f' 表示浮点数 (4字节)
#                 data_frame = struct.pack(
#                     '<BBffffB',
#                     header,
#                     checksum,
#                     x_speed,
#                     y_speed,
#                     rotate,
#                     yaw_speed,
#                     running_state
#                 )
                
#                 # 将打包好的数据帧写入串口
#                 self.serial_conn.write(data_frame)
#                 # 打印发送的数据的十六进制形式，方便调试
#                 self.get_logger().info(f'已发送调试数据帧 (全1): {data_frame.hex()}')

#             except serial.SerialException as e:
#                 self.get_logger().error(f'发送数据时发生错误: {e}')
#         else:
#             self.get_logger().warning('无法发送数据，串口未连接。')

#     def read_serial_data(self):
#         """
#         直接从串口读取数据。
#         此函数由定时器周期性调用。
#         """
#         if self.serial_conn and self.serial_conn.is_open:
#             try:
#                 # 读取一行数据
#                 data = self.serial_conn.readline().decode('utf-8', errors='ignore').strip()
#                 if data:
#                     self.get_logger().info(f"接收到原始数据: {data}")
#                     try:
#                         # 尝试将数据解析为JSON
#                         parsed_data = json.loads(data)
#                         self.process_data(parsed_data)
#                     except (json.JSONDecodeError, ValueError, TypeError) as e:
#                         self.get_logger().error(f'JSON解析失败: {e}')
#                         return
#             except serial.SerialException as e:
#                 self.get_logger().error(f'读取串口数据时发生错误: {e}')
#         else:
#             self.get_logger().warning('串口未连接。')

#     def process_data(self, data):
#         """
#         处理从下位机接收并解析后的数据，然后发布为ROS2消息。
#         """
#         msg = Referee()
#         try:
#             msg.game_type = int(data.get('game_type', 0))
#             msg.game_progress = int(data.get('game_progress', 0))
#             msg.remain_hp = int(data.get('remain_hp', 0))
#             msg.max_hp = int(data.get('max_hp', 0))
#             msg.stage_remain_time = int(data.get('stage_remain_time', 0))
#             msg.bullet_remaining_num_17mm = int(data.get('bullet_remaining_num_17mm', 0))
#             msg.red_outpost_hp = int(data.get('red_outpost_hp', 0))
#             msg.red_base_hp = int(data.get('red_base_hp', 0))
#             msg.blue_outpost_hp = int(data.get('blue_outpost_hp', 0))
#             msg.blue_base_hp = int(data.get('blue_base_hp', 0))
#             msg.rfid_status = int(data.get('rfid_status', 0))
#             # 发布消息
#             self.publisher_.publish(msg)
#             self.get_logger().info('已发布从STM32接收到的数据。')
#         except (ValueError, TypeError) as e:
#             self.get_logger().error(f"处理接收到的数据时出错: {e}")

#     def __del__(self):
#         """
#         节点销毁时的析构函数，用于关闭串口连接。
#         """
#         if self.serial_conn and self.serial_conn.is_open:
#             self.serial_conn.close()
#             self.get_logger().info(f'串口 {self.serial_port} 已关闭。')

# def main(args=None):
#     rclpy.init(args=args)
#     serial_node = SerialNode()
    
#     # 检查节点是否成功初始化
#     if rclpy.ok():
#         try:
#             # rclpy.spin() 会阻塞主线程，使节点保持活动状态以处理定时器等事件
#             rclpy.spin(serial_node)
#         except KeyboardInterrupt:
#             # 允许通过 Ctrl+C 来停止节点
#             serial_node.get_logger().info('检测到键盘中断，正在关闭节点...')
#         finally:
#             # 确保节点被正确销毁
#             serial_node.destroy_node()
#             rclpy.shutdown()

# if __name__ == '__main__':
#     main()








