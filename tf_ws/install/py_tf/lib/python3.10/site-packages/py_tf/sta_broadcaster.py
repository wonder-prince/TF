import math
import rclpy
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped
from tf_transformations import quaternion_from_euler

class TransformBroadcasterNode(Node):
    def __init__(self):
        super().__init__('transform_broadcaster_node')
        
        # 创建变换广播器
        self.static_broadcaster_ = StaticTransformBroadcaster(self)
        
        # 创建定时器，每秒调用一次回调函数发布变换
        #self.timer = self.create_timer(1.0, self.broadcast_transform)
        self.publish_static_tf()
    def publish_static_tf(self):
        # 创建一个TransformStamped消息来存储变换信息
        transform_stamped = TransformStamped()
        transform_stamped.header.stamp = self.get_clock().now().to_msg()
        transform_stamped.header.frame_id = 'base_link'
        transform_stamped.child_frame_id = 'laser'

        # 设置变换的平移部分，假设激光雷达在底盘前方20厘米，高度为10厘米
        transform_stamped.transform.translation.x = 0.5
        transform_stamped.transform.translation.y = 0.3
        transform_stamped.transform.translation.z = 0.6

        # 设置变换的旋转部分,欧拉角转四元数，第一行是欧拉角
        quat = quaternion_from_euler(math.radians(180), 0, 0)
        transform_stamped.transform.rotation.x = quat[0]
        transform_stamped.transform.rotation.y = quat[1]
        transform_stamped.transform.rotation.z = quat[2]
        transform_stamped.transform.rotation.w = quat[3]

        # 发送变换
        #self._broadcaster.sendTransform(transform_stamped)      #动态广播
        self.static_broadcaster_.sendTransform(transform_stamped)       #静态广播

def main(args=None):
    rclpy.init()
    tf_demo = TransformBroadcasterNode()
    rclpy.spin(tf_demo)
    tf_demo.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
