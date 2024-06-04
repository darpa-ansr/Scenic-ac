import logging
import math
import pandas as pd
import tarfile

from mcap.reader import make_reader, McapReader
from mcap_ros2.decoder import DecoderFactory
from pathlib import Path
from typing import Union

from scenic.syntax.veneer import verbosePrint, _globalParameters

logger: logging.Logger = logging.getLogger(__name__)

class BagFileDoesNotExistException(Exception):
    pass

class BagFileIsWrongFormat(Exception):
    pass

class EoIPoseOutsideTimeTolerance(Exception):
    pass

POSE_VALID_TIME_TOLERANCE = 1.5
PERCEPTION_TOPIC = '/adk_node/input/perception'
GT_PERCEPTION_TOPIC = '/adk_node/ground_truth/perception'
GT_ODOMETRY_TOPIC = _globalParameters["ego_topic"]
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

def bag_to_dataframe(bag_path: Union[str, Path],
                                description_info: dict) -> pd.DataFrame:
    """
    Extracts ROS bag (archive) into an intermediate format used by various metrics computation
    classes in .metrics.

    Returns pandas.DataFrame object.
    """
    bag_path = verify_bag_path(bag_path)

    POSE_VALID_TIME_TOLERANCE = 5
    valid_topics = [PERCEPTION_TOPIC, GT_PERCEPTION_TOPIC, GT_ODOMETRY_TOPIC, COLLISION_TOPIC]
    valid_topics += _globalParameters["entity_ground_truth_topics"]
    entity_attribute_map = {}
    entity_last_pose = {}
    for eoi in description_info['scenario_objective']['entities_of_interest']:
        entity_id = eoi['entity_id']
        entity_attribute_map[entity_id] = eoi['attributes']
        entity_last_pose[entity_id] = {'timestamp': None, 'pose': None}

    data = []
    with open(bag_path, 'rb') as f:
        decoders = {}
        bag_reader = make_reader(f, decoder_factories=[DecoderFactory()])
        # TODO: There are many EoI pose messages. Instead of looping through all of them,
        # is there a way to filter down envcar_pose messages that are near ground truth
        # messages?
        # for _, channel, _, message in bag_reader.iter_decoded_messages(topics=valid_topics):
        for schema,channel,encoded_msg in bag_reader.iter_messages(topics=valid_topics):
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
            if channel.topic in [GT_PERCEPTION_TOPIC, PERCEPTION_TOPIC]:
                seconds: float = float(message.detection_time.sec) + (
                        message.detection_time.nanosec / 1e9)
                if channel.topic == GT_PERCEPTION_TOPIC:
                    event = 'GT'
                    pose_timestamp = entity_last_pose[message.entity_id]['timestamp']
                    eoi_pose = entity_last_pose[message.entity_id]['pose']
                    # Warn if detection time is off by more than POSE_VALID_TIME_TOLERANCE seconds
                    # from the ground truth message time stamp
                    if  pose_timestamp is None or abs(pose_timestamp - seconds) > POSE_VALID_TIME_TOLERANCE:
                        logger.warn("No valid pose data available for ground truth report."
                                    f" Not reporting ground truth at {seconds:.3f} s.")
                        continue
                    else:
                        # Use correct EoI position from the corresponding airsim topic
                        pos_x = eoi_pose.position.x
                        pos_y = eoi_pose.position.y
                        pos_z = eoi_pose.position.z
                else:
                    event = 'MSG'
                    pos_x = message.position.position.x
                    pos_y = message.position.position.y
                    pos_z = message.position.position.z

                # Check included for testing Maneuver Thread logic on modified Perception Thread runs.
                # Excludes target perception messages from mission data for Maneuver Thread.
                if event == 'MSG' and description_info["scenario_thread"] == "maneuver_thread":
                    continue
                else:
                    if (message.enter_or_leave != 1) or (channel.topic == PERCEPTION_TOPIC):
                        data.append({
                            'Timestamp': seconds,
                            'Event': event,
                            'EntityID': message.entity_id,
                            'PositionX': pos_x,
                            'PositionY': pos_y,
                            'PositionZ': pos_z,
                            'Type': entity_attribute_map[message.entity_id]['class'],
                            'Color': entity_attribute_map[message.entity_id]['color'],
                            'Confidence': message.probability,
                        })
                # Fake target perception messages for the Maneuver Thread to make COP metrics work.
                if ((description_info["scenario_thread"] ==  "maneuver_thread")
                    and (channel.topic == GT_PERCEPTION_TOPIC)):
                    data.append({
                        'Timestamp': seconds,
                        'Event': 'MSG',
                        'EntityID': message.entity_id,
                        'PositionX': pos_x,
                        'PositionY': pos_y,
                        'PositionZ': pos_z,
                        'Type': entity_attribute_map[message.entity_id]['class'],
                        'Color': entity_attribute_map[message.entity_id]['color'],
                        'Confidence': message.probability,
                    })
            elif channel.topic == GT_ODOMETRY_TOPIC:
                seconds = (float(message.header.stamp.sec)
                                  + (message.header.stamp.nanosec / 1e9))
                data.append({
                    'Timestamp': seconds,
                    'Event': 'ODOM',
                    'WorldX': message.pose.pose.position.x,
                    'WorldY': message.pose.pose.position.y,
                    'WorldZ': message.pose.pose.position.z,
                    'VelocityX': message.twist.twist.linear.x,
                    'VelocityY': message.twist.twist.linear.y,
                    'VelocityZ': message.twist.twist.linear.z
                })
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
            else:
                # Record EoI position from the airsim topic because ground truth topic
                # reports only the initial EoI position.
                entity_id: str = channel.topic.split('/')[2]
                seconds: float = float(message.header.stamp.sec) + (
                        message.header.stamp.nanosec / 1e9)
                if entity_last_pose.get(entity_id, None) is None:
                    continue
                entity_last_pose[entity_id]['timestamp'] = seconds
                last_pose = entity_last_pose[entity_id]['pose']
                # Generate a new GT_POSITION message only when the target position shifts by
                # more than the perception detection error, specified in metrics_config.json.
                # This reduces the number of spurious position update messages, and is also relied
                # upon by the COP latency metric (see cop_latency.py for details).
                if ((entity_last_pose[entity_id]['pose'] is None)
                    or (abs(message.pose.position.x - last_pose.position.x) > 5)
                    or (abs(message.pose.position.y - last_pose.position.y) > 5)
                    or (abs(message.pose.position.z - last_pose.position.z) > 5)):
                    data.append({
                        'Timestamp': seconds,
                        'Event': 'GT_POSITION',
                        'EntityID': entity_id,
                        'PositionX': message.pose.position.x,
                        'PositionY': message.pose.position.y,
                        'PositionZ': message.pose.position.z
                    })
                entity_last_pose[entity_id]['pose'] = message.pose
    return pd.DataFrame.from_dict(data)