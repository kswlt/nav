import rclpy
from rclpy.node import Node
from std_msgs.msg import Int8
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener

class RegionMonitorNode(Node):
    def __init__(self):
        super().__init__('region_monitor_node')

        # ================= 配置区域 =================
        # 定义颠簸路段的区域 (单位: 米)
        self.regions = [
            {'name': 'bump_road_1', 'x_min': 6.6, 'x_max': 11.8, 'y_min': -1.4, 'y_max': 3.34},
            # 您可以添加更多区域...
        ]
        # ===========================================

        # 发布指令的话题
        self.publisher_ = self.create_publisher(Int8, '/cmd_chassis_mode', 10)
        
        # [新增] 发布可视化 Marker 的话题
        self.marker_pub = self.create_publisher(MarkerArray, '/region_markers', 10)
        
        # TF 监听器
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        # 状态记录
        self.current_mode = -1 
        
        # 定时器 10Hz
        self.timer = self.create_timer(0.1, self.timer_callback)
        self.get_logger().info('区域监控节点已启动 (带可视化功能)')

    def timer_callback(self):
        try:
            # 1. 获取机器人坐标
            t = self.tf_buffer.lookup_transform(
                'map',
                'base_footprint', 
                rclpy.time.Time())
            
            current_x = t.transform.translation.x
            current_y = t.transform.translation.y

            # 2. 检查区域并发布可视化
            # 我们将在这里同时处理“逻辑判断”和“可视化绘制”
            in_any_region = False
            marker_array = MarkerArray()
            timestamp = self.get_clock().now().to_msg()

            for i, region in enumerate(self.regions):
                # 判断机器人是否在这个具体的区域内
                is_inside_this = (region['x_min'] <= current_x <= region['x_max'] and 
                                  region['y_min'] <= current_y <= region['y_max'])
                
                if is_inside_this:
                    in_any_region = True

                # === 构建可视化 Marker ===
                marker = Marker()
                marker.header.frame_id = "map"
                marker.header.stamp = timestamp
                marker.ns = "bump_regions"
                marker.id = i
                marker.type = Marker.LINE_STRIP  # 线条框
                marker.action = Marker.ADD
                marker.scale.x = 0.05  # 线条宽度
                
                # 颜色逻辑：进入变红(1.0, 0.0, 0.0)，平时绿(0.0, 1.0, 0.0)
                if is_inside_this:
                    marker.color.r = 1.0; marker.color.g = 0.0; marker.color.b = 0.0
                else:
                    marker.color.r = 0.0; marker.color.g = 1.0; marker.color.b = 0.0
                marker.color.a = 1.0  # 不透明度

                # 设置矩形的4个顶点 (首尾相连需要5个点)
                p1 = Point(x=region['x_min'], y=region['y_min'], z=0.0)
                p2 = Point(x=region['x_max'], y=region['y_min'], z=0.0)
                p3 = Point(x=region['x_max'], y=region['y_max'], z=0.0)
                p4 = Point(x=region['x_min'], y=region['y_max'], z=0.0)

                marker.points = [p1, p2, p3, p4, p1] # 闭合回路
                
                # 设置生命周期 (比定时器稍长，防止闪烁)
                marker.lifetime.sec = 0
                marker.lifetime.nanosec = 1200000000 # 0.2s

                marker_array.markers.append(marker)

            # 发布可视化 Marker
            self.marker_pub.publish(marker_array)
            
            # 3. 发送控制指令 (原有逻辑)
            if in_any_region:
                self.publish_mode(1)
                if self.current_mode != 1:
                    self.get_logger().warn(f'>>> 进入区域 -> 发送 1 (对齐模式)')
                    self.current_mode = 1
            else:
                self.publish_mode(0)
                if self.current_mode != 0:
                    self.get_logger().info(f'<<< 离开区域 -> 发送 0 (正常模式)')
                    self.current_mode = 0

        except TransformException:
            # TF 还没准备好，暂时不处理
            pass

    def publish_mode(self, mode):
        msg = Int8()
        msg.data = mode
        self.publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = RegionMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()