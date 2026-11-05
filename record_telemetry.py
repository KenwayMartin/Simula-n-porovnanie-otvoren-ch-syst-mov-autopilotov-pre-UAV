import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Imu
import csv
from datetime import datetime

class TelemetryRecorder(Node):
    def __init__(self):
        super().__init__('telemetry_recorder')
        
        # Створюємо файл для запису
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.filename = f"telemetry_px4_{timestamp}.csv"
        
        self.file = open(self.filename, 'w', newline='')
        self.writer = csv.writer(self.file)
        
        # Заголовки CSV
        self.writer.writerow([
            'Time', 'X', 'Y', 'Z', 
            'Ori_X', 'Ori_Y', 'Ori_Z', 'Ori_W',
            'Accel_X', 'Accel_Y', 'Accel_Z'
        ])
        
        self.get_logger().info(f"Запис телеметрії розпочато у файл: {self.filename}")

        # QoS профіль для сумісності з MAVROS (BEST_EFFORT)
        qos_profile = QoSProfile(depth=10)
        qos_profile.reliability = ReliabilityPolicy.BEST_EFFORT

        # Підписка на позицію
        self.pose_sub = self.create_subscription(
            PoseStamped,
            '/mavros/local_position/pose',
            self.pose_callback,
            qos_profile  # <-- Використовуємо правильний QoS
        )

        # Підписка на IMU
        self.imu_sub = self.create_subscription(
            Imu,
            '/mavros/imu/data',
            self.imu_callback,
            qos_profile  # <-- Використовуємо правильний QoS
        )

        self.current_pose = None
        self.current_imu = None

    def pose_callback(self, msg):
        self.current_pose = msg
        self.save_data()

    def imu_callback(self, msg):
        self.current_imu = msg
        self.save_data()

    def save_data(self):
        if self.current_pose is not None and self.current_imu is not None:
            row = [
                datetime.now().strftime("%H:%M:%S.%f")[:-3],
                round(self.current_pose.pose.position.x, 4),
                round(self.current_pose.pose.position.y, 4),
                round(self.current_pose.pose.position.z, 4),
                round(self.current_pose.pose.orientation.x, 4),
                round(self.current_pose.pose.orientation.y, 4),
                round(self.current_pose.pose.orientation.z, 4),
                round(self.current_pose.pose.orientation.w, 4),
                round(self.current_imu.linear_acceleration.x, 4),
                round(self.current_imu.linear_acceleration.y, 4),
                round(self.current_imu.linear_acceleration.z, 4)
            ]
            self.writer.writerow(row)
            self.file.flush()

    def destroy_node(self):
        self.file.close()
        self.get_logger().info(f"Запис завершено. Файл збережено: {self.filename}")
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    recorder = TelemetryRecorder()
    try:
        rclpy.spin(recorder)
    except KeyboardInterrupt:
        pass
    finally:
        recorder.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()