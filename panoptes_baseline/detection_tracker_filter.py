import rclpy
from rclpy.node import Node
from depthai_ros_msgs.msg import TrackDetection2DArray
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class DetectionTrackerFilterNode(Node):
    def __init__(self):
        super().__init__('detection_filter_node')

        # Declare input_topic parameter with default value
        self.declare_parameter('input_topic', '')
        self.declare_parameter('namespace', '')
        self.declare_parameter('max_humans', 0)

        # Get the parameter value
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        self.namespace = self.get_parameter('namespace').get_parameter_value().string_value
        self.max_humans = self.get_parameter('max_humans').get_parameter_value().integer_value

        # Subscriber
        self.subscription = self.create_subscription(
            TrackDetection2DArray,
            input_topic,
            self.listener_callback,
            10)

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

    def listener_callback(self, msg):
        human_id = 0
        for detection in msg.detections:
            result = detection.results[0]
            if int(result.hypothesis.class_id) == 0 and int(detection.tracking_status) < 2 and (human_id < self.max_humans):
                self.broadcast_transform(result, human_id, msg.header.frame_id, msg.header.stamp)
                human_id += 1

    def broadcast_transform(self, result, human_id, parent_frame, stamp):
        # Create a TransformStamped message
        t = TransformStamped()

        # Set the timestamp and frame info
        t.header.stamp = stamp
        t.header.frame_id = parent_frame
        t.child_frame_id = f"{self.namespace}/human_{human_id}"

        # Set the transform using the detection's position    
        t.transform.translation.x = result.pose.pose.position.x
        t.transform.translation.y = - result.pose.pose.position.y
        t.transform.translation.z = result.pose.pose.position.z
        t.transform.rotation.x = result.pose.pose.orientation.x
        t.transform.rotation.y = result.pose.pose.orientation.y
        t.transform.rotation.z = result.pose.pose.orientation.z
        t.transform.rotation.w = result.pose.pose.orientation.w

        # Broadcast the transform
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = DetectionTrackerFilterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
