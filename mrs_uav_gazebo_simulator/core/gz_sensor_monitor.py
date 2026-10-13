import re
import threading
import time
import xml.dom.minidom

import gz.transport13 as gz_transport

# Rclpy imports
from rclpy.node import Node

# default sensor topic: /world/<world>/model/<model>/link/<link>/sensor/<sensor>/<data>
SENSOR_TOPIC = re.compile(r'^/world/[^/]+/model/([^/]+)/link/([^/]+)/sensor/([^/]+)/[^/]+$')

# accept any message type
GENERIC_MSG_TYPE = 'google.protobuf.Message'


# #{ sdf_sensors(sdf_content)
def sdf_sensors(sdf_content: str) -> set[tuple[str, str]]:
    '''
    :return: (link, sensor) names of the sensors publishing on their default topic
    '''
    sensors = set()
    for sensor in xml.dom.minidom.parseString(sdf_content).getElementsByTagName('sensor'):
        has_topic = any(child.nodeName == 'topic' for child in sensor.childNodes)
        link = sensor.parentNode
        if not has_topic and link.nodeName == 'link':
            sensors.add((link.getAttribute('name'), sensor.getAttribute('name')))
    return sensors


# #}


class GzSensorMonitor():
    '''
    Tells when the Gazebo sensors of a spawned model publish data, the firmware is launched only then.

    gz-transport registers a subscriber at the publisher by a single UDP multicast message without
    any retry. It can get lost in the burst of discovery traffic after a model is spawned, the
    subscription then never receives data (PX4 misses e.g. its barometer and refuses to arm).
    Subscriptions that stay silent are therefore renewed.
    '''

    # #{ __init__(self, ros_node, resubscribe_period)
    def __init__(self, ros_node: Node, resubscribe_period: float = 2.0):
        self._ros_node = ros_node
        self._resubscribe_period = resubscribe_period

        self._gz_node = gz_transport.Node()
        self._lock = threading.Lock()

        # model -> sensors (link, sensor) still waiting for a subscription with data
        self._expected = {}
        # topic -> [model, sensor, received data, time of subscription]
        self._topics = {}

    # #}

    # #{ add(self, model, sensors)
    def add(self, model: str, sensors: set[tuple[str, str]]):
        self._expected[model] = set(sensors)

    # #}

    # #{ update(self)
    def update(self):
        '''
        Subscribes to new sensor topics of the monitored models and renews silent subscriptions.
        '''
        if not self._expected:
            return

        # every topic_list() call adds discovery traffic, so it is called once for all models
        for topic in self._gz_node.topic_list():
            match = SENSOR_TOPIC.match(topic)
            if not match or topic in self._topics:
                continue

            model, link, sensor = match.groups()
            if (link, sensor) in self._expected.get(model, ()):
                self._subscribe(topic, model, (link, sensor))

        with self._lock:
            silent = [t for t, (_, _, received, since) in self._topics.items()
                      if not received and time.monotonic() - since >= self._resubscribe_period]

        for topic in silent:
            self._ros_node.get_logger().info(f'No data on {topic} yet, subscribing again')
            self._gz_node.unsubscribe(topic)
            model, sensor, _, _ = self._topics[topic]
            self._subscribe(topic, model, sensor)

    # #}

    # #{ missing(self, model) -> list[str]
    def missing(self, model: str) -> list[str]:
        '''
        :return: sensors of the model that did not publish yet, empty when all did
        '''
        with self._lock:
            received = {sensor for m, sensor, got, _ in self._topics.values() if m == model and got}

        return sorted(f'{link}/{sensor}' for link, sensor in self._expected.get(model, set()) - received)

    # #}

    # #{ release(self, model)
    def release(self, model: str):
        self._expected.pop(model, None)

        with self._lock:
            topics = [t for t, (m, _, _, _) in self._topics.items() if m == model]
            for topic in topics:
                del self._topics[topic]

        for topic in topics:
            self._gz_node.unsubscribe(topic)

    # #}

    # #{ shutdown(self)
    def shutdown(self):
        # no subscription callbacks may run during interpreter shutdown
        for model in list(self._expected):
            self.release(model)

    # #}

    # #{ _subscribe(self, topic, model, sensor)
    def _subscribe(self, topic: str, model: str, sensor: tuple[str, str]):
        with self._lock:
            self._topics[topic] = [model, sensor, False, time.monotonic()]

        self._gz_node.subscribe_raw(topic, lambda data, info: self._callback(topic), GENERIC_MSG_TYPE,
                                    gz_transport.SubscribeOptions())

    # #}

    # #{ _callback(self, topic)
    def _callback(self, topic: str):
        # also called by a renewed or released subscription, stale calls are ignored
        with self._lock:
            if topic in self._topics:
                self._topics[topic][2] = True

    # #}
