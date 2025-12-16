from sensor_msgs.msg import TimeReference
from std_msgs.msg import Header


class ImgLatencyManager:
    def __init__(self, img: str, frame_id: str = "teleop", node=None):
        self.img = img
        self.frame_id = frame_id
        self.last_header = None
        self.node = node

        if node is not None:
            self.sub = node.create_subscription(Header, f"{img}/header", self.callback, 10)
            self.pub = node.create_publisher(TimeReference, f"{img}/latency", 10)

    def callback(self, header: Header):
        if self.node is None:
            return

        latency = TimeReference()
        latency.header = Header()
        latency.header.stamp = self.node.get_clock().now().to_msg()
        latency.header.frame_id = self.frame_id
        latency.time_ref = header.stamp
        latency.source = self.img

        self.pub.publish(latency)
