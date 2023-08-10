"""
Convert between Apollo and Applied data formats for the traffic lights.
"""

# Apollo imports
from modules.perception.proto import traffic_light_detection_pb2

# Simian actor msg
from simian.public.osi3 import osi_trafficlight_pb2


def _convert_osi_color_to_apollo_color(osi_color):
    if osi_color == osi_trafficlight_pb2.TrafficLight.Classification.COLOR_RED:
        return traffic_light_detection_pb2.TrafficLight.RED
    elif osi_color == osi_trafficlight_pb2.TrafficLight.Classification.COLOR_YELLOW:
        return traffic_light_detection_pb2.TrafficLight.YELLOW
    elif osi_color == osi_trafficlight_pb2.TrafficLight.Classification.COLOR_GREEN:
        return traffic_light_detection_pb2.TrafficLight.GREEN
    # Unknown color or not supported by Apollo
    return traffic_light_detection_pb2.TrafficLight.UNKNOWN


def _set_traffic_light_info(traffic_light, out_msg):
    out_msg.color = _convert_osi_color_to_apollo_color(traffic_light.classification.color)
    # OSI identifier are ints while apollo uses strings
    out_msg.id = str(traffic_light.id.value)
    out_msg.confidence = 1.0
    out_msg.tracking_time = 0.0


def create_traffic_light_detection_msg(sim_time, traffic_lights):
    """Creates Apollo TrafficLightDetection message

    For the header, it only fills in timestamp_sec. In particular,
    sequence number and module name are left empty.

    For the payload, we do not fill traffic_light_debug msg
    """
    traffic_light_detection_msg = traffic_light_detection_pb2.TrafficLightDetection()
    traffic_light_detection_msg.header.timestamp_sec = sim_time
    for traffic_light in traffic_lights:
        MODE_CONSTANT = osi_trafficlight_pb2.TrafficLight.Classification.MODE_CONSTANT
        if traffic_light.classification and traffic_light.classification.mode == MODE_CONSTANT:
            out = traffic_light_detection_msg.traffic_light.add()
            _set_traffic_light_info(traffic_light, out)

    if len(traffic_light_detection_msg.traffic_light) > 0:
        traffic_light_detection_msg.contain_lights = True

    return traffic_light_detection_msg
