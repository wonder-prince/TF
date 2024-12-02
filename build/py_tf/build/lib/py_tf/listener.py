import rclpy
from rclpy.node import Node
import rclpy.time
from tf2_ros import TransformListener, Buffer
from tf_transformations import euler_from_quaternion

class TransformListenerNode(Node):
    def __init__(self):
        super().__init__('transform_listener_node')

        # 创建一个Buffer对象来存储变换
        self.tf_buffer = Buffer()
        
        # 创建TransformListener来监听变换
        self.tf_listener = TransformListener(self.tf_buffer, self)
        
        # 创建定时器，每秒调用一次回调函数
        self.timer = self.create_timer(1.0, self.on_timer)
        
    def on_timer(self):
        try:
            # 查询从'laser'到'base_link'的变换
            results = self.tf_buffer.lookup_transform('base_link', 'laser', rclpy.time.Time(seconds=0),rclpy.time.Duration(seconds=1))
            transform =results.transform        #收到的发布中包含平移和旋转四元数，但是欧拉角需要进行一个变换
            # 将激光雷达坐标系下的点转换到底盘坐标系下
            #transformed_point = tf2_geometry_msgs.do_transform_point(self.laser_point, transform)
            rotation_euler = euler_from_quaternion([
                transform.rotation.x,
                transform.rotation.y,
                transform.rotation.z,
                transform.rotation.w
            ])
            # 打印转换后的点
            self.get_logger().info(f'平移:{transform.translation},旋转四元数:{transform.rotation},旋转欧拉角:{rotation_euler}')
        
        except Exception as e:
            self.get_logger().warn(f'Could not transform laser point: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = TransformListenerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
