import rclpy
from rclpy.node import Node

from scapy.all import sniff, Raw
import asn1tools

from threading import Thread

import os

class V2Xbridge(Node):

    def __init__(self):
        super().__init__("tv2x_bridge")

        self.load_parameters()
        self.cam = asn1tools.compile_files(self.asn_file_cam, "uper")
        self.denm = asn1tools.compile_files(self.asn_file_denm, "uper")

        self.tshark = Thread(self.start_listener_v2x, daemon=True)
        self.tshark.start()
        

    def load_parameters(self):
        self.declare_parameter("general.v2x_interface", "")
        self.v2x_interface = self.get_parameter("general.v2x_interface").value

        self.declare_parameter("general.asn_file_cam", "")
        self.asn_file_cam = self.get_parameter("general.asn_file_cam").value

        self.declare_parameter("general.asn_file_denm", "")
        self.asn_file_denm = self.get_parameter("general.asn_file_denm").value

        self.declare_parameter("topic.cam", "")
        self.topic_cam = self.get_parameter("topic.cam").value
        
        self.declare_parameter("topic.denm", "")
        self.topic_denm = self.get_parameter("topic.denm").value

    def start_listener_v2x(self):
        sniff(iface=self.v2x_interface, prn=self.cam_callback, count=10)

    def cam_callback(self, packet):
        if packet.haslayer(Raw):
            raw_data = packet[Raw].load
            try:
                # Decode the UPER-encoded CAM message
                decoded_data = self.cam.decode('CAM', raw_data)
                self.get_logger().info(f"Decoded CAM Message: {decoded_data}")
            except Exception as e:
                self.get_logger().info(f"Error decoding packet: {e}")


def main(args=None):
    rclpy.init(args=args)

    try:
        tv2x_bridge = V2Xbridge()
        rclpy.spin(tv2x_bridge)
    except KeyboardInterrupt:
        pass
    
    if tv2x_bridge is not None:
        tv2x_bridge.destroy_node()
    
    if rclpy.ok():
        rclpy.shutdown()