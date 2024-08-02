# Copyright 2024 The Johns Hopkins University Applied Physics Laboratory LLC

import json
import logging
import math
import pandas as pd
import re
import sys
import tarfile

from mcap.reader import make_reader
from mcap_ros2.decoder import DecoderFactory
from pathlib import Path
from scipy.spatial.transform import Rotation
import scipy.linalg as linalg
from typing import Union

logger: logging.Logger = logging.getLogger(__name__)

class BagFileDoesNotExistException(Exception):
    pass

class BagFileIsWrongFormat(Exception):
    pass

class EoIPoseOutsideTimeTolerance(Exception):
    pass

POSE_VALID_TIME_TOLERANCE = 1.5
PERCEPTION_TOPIC = '/adk_node/input/perception'
GT_ODOMETRY_TOPIC = '/adk_node/SimpleFlight/odom_local_ned'
COLLISION_TOPIC = '/adk_node/SimpleFlight/collision_state'

def verify_bag_path(bag_path: Union[str, Path]) -> Path:
    """
    Checks whether a .tgz bag archive or .mcap bag file is present.

    Returns Path to whichever file is present, preferring .mcap if both are present.
    """
    if not isinstance(bag_path, Path):
        bag_path = Path(bag_path)

    if not bag_path.exists():
        raise BagFileDoesNotExistException

    print(f"bag_path={bag_path}")
    is_mcap = bag_path.suffix == '.mcap'
    is_tarfile = bag_path.suffix == '.tgz' and tarfile.is_tarfile(bag_path)

    if not is_mcap:
        if not is_tarfile:
            raise BagFileIsWrongFormat
        else:
            tar = tarfile.open(bag_path, 'r')
            tar.extractall(path=bag_path.parent)
            bag_path = Path(bag_path.parent / 'bags_0.mcap')

            if not bag_path.exists():
                raise BagFileDoesNotExistException
    return bag_path

def bag_to_dataframe(bag_path: Union[str, Path], topics: list, entity_attribute_map: dict) -> pd.DataFrame:
    """
    Extracts ROS bag (archive) into an intermediate format used by various metrics computation
    classes in .metrics.

    Returns pandas.DataFrame object.
    """
    bag_path = verify_bag_path(bag_path)

    data = []
    with open(bag_path, 'rb') as f:
        decoders: dict = {}
        bag_reader = make_reader(f, decoder_factories=[DecoderFactory()])
        for schema,channel,encoded_msg in bag_reader.iter_messages(topics=topics):
            if channel.topic not in decoders:
                decoders[channel.topic] = DecoderFactory().decoder_for('cdr', schema)
            try:
                message = decoders[channel.topic](encoded_msg.data)
            except Exception as E:
                if channel.topic == '/adk_node/input/perception':
                    # Some performers are using the old definition, but still are managing to publish...
                    schema_dat = schema.data.decode()
                    new_schema = schema_dat.replace('sensor_msgs/Image image\n','')
                    new_schema = new_schema.replace('Box2D bounding_box2d\n','')
                    schema.data = new_schema.encode()
                    decoders[channel.topic] = DecoderFactory().decoder_for('cdr',schema)
                    message = decoders[channel.topic](encoded_msg.data)
                else:
                    logger.warning(f"Failed to decode a message on {channel.topic}")
                    continue
            # Process perception topics
            if channel.topic == PERCEPTION_TOPIC:
                seconds: float = float(message.detection_time.sec) + (
                        message.detection_time.nanosec / 1e9)
                if message.enter_or_leave != 1:
                    q = message.position.orientation
                    if linalg.norm([q.x, q.y, q.z, q.w]) > 0:
                        R, P, Y = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz")
                    else:
                        R, P, Y = (0.0, 0.0, 0.0)
                    data.append({
                        'Timestamp': seconds,
                        'Event': 'MSG',
                        'EntityID': message.entity_id,
                        'X': message.position.position.x,
                        'Y': message.position.position.y,
                        'Z': message.position.position.z,
                        'Roll': R,
                        'Pitch': P,
                        'Yaw': Y,
                        'Type': entity_attribute_map[message.entity_id]['class'],
                        'Color': entity_attribute_map[message.entity_id]['color'],
                        'Confidence': message.probability,
                    })
            # Process UAV odometry
            elif channel.topic == GT_ODOMETRY_TOPIC:
                seconds = (float(message.header.stamp.sec)
                                  + (message.header.stamp.nanosec / 1e9))
                q = message.pose.pose.orientation
                R, P, Y = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz")
                data.append({
                    'Timestamp': seconds,
                    'Event': 'ODOM',
                    'X': message.pose.pose.position.x,
                    'Y': message.pose.pose.position.y,
                    'Z': message.pose.pose.position.z,
                    'Roll': R,
                    'Pitch': P,
                    'Yaw': Y,
                    'Xdot': message.twist.twist.linear.x,
                    'Ydot': message.twist.twist.linear.y,
                    'Zdot': message.twist.twist.linear.z,
                    'Surge': message.twist.twist.angular.x,
                    'Sway': message.twist.twist.angular.y,
                    'Heave': message.twist.twist.angular.z
                })
            # Process collisions
            elif channel.topic == COLLISION_TOPIC:
                seconds = (float(message.timestamp.sec)
                                  + (message.timestamp.nanosec / 1e9))
                if message.has_collided:
                    data.append({
                        'Timestamp': seconds,
                        'Event': 'CLSN',
                        'Object': message.object_name,
                        'ObjectID': message.object_id
                    })
            # Entity ground truth topics
            else:
                entity_id = channel.topic.split('/')[2]
                seconds = float(message.header.stamp.sec) + (
                        message.header.stamp.nanosec / 1e9)
                q = message.pose.orientation
                R, P, Y = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_euler("xyz")
                data.append({
                    'Timestamp': seconds,
                    'Event': 'GT_POSITION',
                    'EntityID': entity_id,
                    'X': message.pose.position.x,
                    'Y': message.pose.position.y,
                    'Z': message.pose.position.z,
                    'Roll': R,
                    'Pitch': P,
                    'Yaw': Y
                })
    return pd.DataFrame.from_dict(data)