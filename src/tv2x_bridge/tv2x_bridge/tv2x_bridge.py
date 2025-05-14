import rclpy
from rclpy.node import Node

import asn1tools
from pyv2x.v2x_network import V2xNetwork
from pyv2x.etsi import ETSI
from pyv2x.v2x_utils import GeoNetworking

from rclpy.parameter import Parameter

from tv2x_base.msg import GpsSpeed, NavSatHeading
from message_filters import Subscriber, ApproximateTimeSynchronizer

import os

class V2Xbridge(Node):

    def __init__(self):
        super().__init__("tv2x_bridge")

        self.load_parameters()

        self.cam = asn1tools.compile_files(self.asn_file_cam, "uper")
        self.denm = asn1tools.compile_files(self.asn_file_denm, "uper")

        self.net = V2xNetwork(interface=self.v2x_interface)

        timer_period = 0.1 # seconds -> 10 Hz
        timer_cam = self.create_timer(timer_period, self.timer_callback)
        timer_msg = self.create_timer(timer_period, self.receving_msg)


        self.lat, self.long, self.heading, self.speed = 0, 0, 0, 0
        mac_address = get_mac(self.v2x_interface)

        # tv2x_subscription
        gps_subscriber, nav_subscriber = Subscriber(self, GpsSpeed, self.topic_gps), Subscriber(self, NavSatHeading, self.topic_nav)

        # synch the callback
        ats = ApproximateTimeSynchronizer([gps_subscriber, nav_subscriber], queue_size=10, slop=0.1)
        ats.registerCallback(self.v2x_information_callback)
        
    def get_param(self, name: str, default) -> Parameter:
        self.declare_parameter(name, default)
        param = self.get_parameter(name)
        self.get_logger().info(f"param {name} has value {param.value}")
        return param

    def load_parameters(self):
        
        self.v2x_interface = self.get_param("general.v2x_interface", "").value
        self.asn_file_cam = self.get_param("general.asn_file_cam", "").value
        self.asn_file_denm = self.get_param("general.asn_file_denm", "").value

        self.topic_cam = self.get_param("topic.cam", "")
        self.topic_denm = self.get_param("topic.denm", "")
        self.topic_gps = self.get_param("topic.gps", "")
        self.topic_nav = self.get_param("topic.nav", "")

    def v2x_information_callback(self, gps_speed, nav_sat_heading):

        self.lat, self.long = nav_sat_heading.gps_data.latitude, nav_sat_heading.gps_data.longitude
        self.heading = nav_sat_heading.gps_speed
        self.speed = gps_speed.gps_speed_mps

    def timer_callback(self):
        
        timestamp = GeoNetworking.get_gn_timestamp()

        cam_msg = ETSI.new_cam(
            self.cam, gn_addr_address=self.mac_address, station_id=4935, latitude=self.lat, longitude=self.long,
            delta_time=timestamp, speed=self.speed, heading=self.heading
        )

        self.net.send_msg(cam_msg)

    def receving_msg(self):
        pass


def get_mac(interface: str) -> str:
    
    path = f"/sys/class/net/{interface}/address"
    if not os.path.exists(path):
        return "00:00:00:00:00:00"
    
    with open(path, "r+") as f:
        mac = f.read().strip()
    
    return mac


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