import rclpy
from rclpy.node import Node
from tram_vehicle_msgs.msg import VelocitySensor, DriverControllerCommand
from nav_msgs.msg import Odometry
import time


def stamp_to_sec(stamp):
    return stamp.sec + stamp.nanosec * 1e-9


class ReserveOdometryNode(Node):
    def __init__(self):
        super().__init__('reserve_odometry_node')

        self.declare_parameter('model_gain_accel', 0.05)
        self.declare_parameter('model_gain_brake', 0.08)
        self.declare_parameter('model_weight', 0.3)
        self.declare_parameter('slip_threshold', 0.5)

        self.k_accel = self.get_parameter('model_gain_accel').value
        self.k_brake = self.get_parameter('model_gain_brake').value
        self.model_weight = self.get_parameter('model_weight').value
        self.slip_threshold = self.get_parameter('slip_threshold').value
        self.declare_parameter('max_speed', 20.0)
        self.declare_parameter('base_velocity_variance', 0.01)
        self.declare_parameter('slip_variance_gain', 1.0)
        self.declare_parameter('resistance_coefficient', 0.003)
        self.declare_parameter('initial_position', 0.0)
        self.max_speed = self.get_parameter('max_speed').value
        self.base_velocity_variance = self.get_parameter('base_velocity_variance').value
        self.slip_variance_gain = self.get_parameter('slip_variance_gain').value
        self.resistance_coefficient = self.get_parameter('resistance_coefficient').value
        self.initial_position = self.get_parameter('initial_position').value
        
        self.position_x = self.initial_position
        self.model_velocity = 0.0
        self.last_front_stamp = None
        self.last_controller_pos = 0
        self.rear_velocity = None
        self.latency_samples = []
        self.latency_report_interval = 200
        self.position_variance = 0.0

        self.create_subscription(VelocitySensor, '/vehicle/front_bogie_velocity', self.on_front, 10)
        self.create_subscription(VelocitySensor, '/vehicle/rear_bogie_velocity', self.on_rear, 10)
        self.create_subscription(DriverControllerCommand, '/vehicle/driver_position_cmd', self.on_controller, 10)

        self.velocity_pub = self.create_publisher(VelocitySensor, '/result/velocity', 10)
        self.position_pub = self.create_publisher(Odometry, '/result/position', 10)

    def on_controller(self, msg):
        self.last_controller_pos = msg.position

    def on_rear(self, msg):
        self.rear_velocity = msg.velocity / 3.6

    def on_front(self, msg):
        t_start = time.perf_counter()
        stamp = msg.header.stamp
        now = stamp_to_sec(stamp)

        pos = self.last_controller_pos
        traction_rate = self.k_accel * pos if pos >= 0 else self.k_brake * pos
        resistance_rate = -self.resistance_coefficient * self.model_velocity ** 2
        rate = traction_rate + resistance_rate

        dt = 0.0
        if self.last_front_stamp is not None:
            dt = now - self.last_front_stamp
            if dt < 0 or dt > 1.0:
                dt = 0.0
        self.last_front_stamp = now

        self.model_velocity = min(self.max_speed, max(0.0, self.model_velocity + rate * dt))

        front_velocity_ms = msg.velocity / 3.6  
        odometry_velocity = front_velocity_ms
        slip_suspected = False
        if self.rear_velocity is not None:
            mismatch = abs(front_velocity_ms - self.rear_velocity)
            slip_suspected = mismatch > self.slip_threshold
            odometry_velocity = (msg.velocity + self.rear_velocity) / 2.0

        w = self.model_weight + (0.4 if slip_suspected else 0.0)
        w = min(w, 1.0)
        estimated_velocity = w * self.model_velocity + (1.0 - w) * odometry_velocity
        if self.rear_velocity is not None:
            velocity_variance = self.base_velocity_variance + self.slip_variance_gain * mismatch ** 2
        else:
            velocity_variance = self.base_velocity_variance * 10.0  

        self.position_variance += velocity_variance * dt ** 2

        self.position_x += estimated_velocity * dt

        vel_msg = VelocitySensor()
        vel_msg.header.stamp = stamp
        vel_msg.header.frame_id = 'base_link'
        vel_msg.velocity = estimated_velocity
        self.velocity_pub.publish(vel_msg)

        odom_msg = Odometry()
        odom_msg.header.stamp = stamp
        odom_msg.header.frame_id = 'odom'
        odom_msg.child_frame_id = 'base_link'
        odom_msg.pose.pose.position.x = self.position_x
        odom_msg.twist.twist.linear.x = estimated_velocity
        odom_msg.twist.covariance[0] = velocity_variance
        odom_msg.pose.covariance[0] = self.position_variance
        self.position_pub.publish(odom_msg)
        
        latency_ms = (time.perf_counter() - t_start) * 1000.0
        self.latency_samples.append(latency_ms)
        if len(self.latency_samples) >= self.latency_report_interval:
            avg_latency = sum(self.latency_samples) / len(self.latency_samples)
            max_latency = max(self.latency_samples)
            self.get_logger().info(
                f'Задержка обработки за последние {len(self.latency_samples)} сообщений: '
                f'среднее={avg_latency:.3f} мс, максимум={max_latency:.3f} мс'
            )
            self.latency_samples.clear()


def main():
    rclpy.init()
    node = ReserveOdometryNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()