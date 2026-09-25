import rclpy
from rclpy.node import Node
from tram_vehicle_msgs.msg import VelocitySensor, DriverControllerCommand
from nav_msgs.msg import Odometry


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

        self.position_x = 0.0
        self.model_velocity = 0.0
        self.last_front_stamp = None
        self.last_controller_pos = 0
        self.rear_velocity = None

        self.create_subscription(VelocitySensor, '/vehicle/front_bogie_velocity', self.on_front, 10)
        self.create_subscription(VelocitySensor, '/vehicle/rear_bogie_velocity', self.on_rear, 10)
        self.create_subscription(DriverControllerCommand, '/vehicle/driver_position_cmd', self.on_controller, 10)

        self.velocity_pub = self.create_publisher(VelocitySensor, '/result/velocity', 10)
        self.position_pub = self.create_publisher(Odometry, '/result/position', 10)

    def on_controller(self, msg):
        self.last_controller_pos = msg.position

    def on_rear(self, msg):
        self.rear_velocity = msg.velocity

    def on_front(self, msg):
        stamp = msg.header.stamp
        now = stamp_to_sec(stamp)

        pos = self.last_controller_pos
        rate = self.k_accel * pos if pos >= 0 else self.k_brake * pos

        dt = 0.0
        if self.last_front_stamp is not None:
            dt = now - self.last_front_stamp
            if dt < 0 or dt > 1.0:
                dt = 0.0
        self.last_front_stamp = now

        self.model_velocity = max(0.0, self.model_velocity + rate * dt)

        odometry_velocity = msg.velocity
        slip_suspected = False
        if self.rear_velocity is not None:
            mismatch = abs(msg.velocity - self.rear_velocity)
            slip_suspected = mismatch > self.slip_threshold
            odometry_velocity = (msg.velocity + self.rear_velocity) / 2.0

        w = self.model_weight + (0.4 if slip_suspected else 0.0)
        w = min(w, 1.0)
        estimated_velocity = w * self.model_velocity + (1.0 - w) * odometry_velocity

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
        self.position_pub.publish(odom_msg)


def main():
    rclpy.init()
    node = ReserveOdometryNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()