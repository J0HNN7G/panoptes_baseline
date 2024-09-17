import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

class DetectionFilterNode(Node):
    def __init__(self):
        super().__init__('detection_filter_node')

        # Declare input_topic parameter with default value
        self.declare_parameter('input_topic', '/robomaster_20/oak/nn/spatial_detections')

        # Get the parameter value
        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value

        # Subscriber
        self.subscription = self.create_subscription(
            Detection3DArray,
            input_topic,  # Use the input topic from the parameter
            self.listener_callback,
            10)

        # TF Broadcaster
        self.tf_broadcaster = TransformBroadcaster(self)

    def listener_callback(self, msg):
        # Go through the detections and publish transforms for humans (class_id == 15)
        for i, detection in enumerate(msg.detections):
            for result in detection.results:
                if int(result.hypothesis.class_id) == 15:
                    # Broadcast transform for each detected human
                    self.broadcast_transform(result, i, msg.header.frame_id, msg.header.stamp)

    def broadcast_transform(self, result, human_id, parent_frame, stamp):
        # Create a TransformStamped message
        t = TransformStamped()

        # Set the timestamp and frame info
        t.header.stamp = stamp
        t.header.frame_id = parent_frame
        t.child_frame_id = f"robomaster_20/human_{human_id}"

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
    node = DetectionFilterNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
