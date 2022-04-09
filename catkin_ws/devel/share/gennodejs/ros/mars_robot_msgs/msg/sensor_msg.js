// Auto-generated. Do not edit!

// (in-package mars_robot_msgs.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class sensor_msg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.laser_top_hit = null;
      this.laser_left_hit = null;
      this.laser_right_hit = null;
      this.depth_bottom_switch = null;
      this.depth_top_switch = null;
      this.yaw = null;
      this.mass = null;
    }
    else {
      if (initObj.hasOwnProperty('laser_top_hit')) {
        this.laser_top_hit = initObj.laser_top_hit
      }
      else {
        this.laser_top_hit = false;
      }
      if (initObj.hasOwnProperty('laser_left_hit')) {
        this.laser_left_hit = initObj.laser_left_hit
      }
      else {
        this.laser_left_hit = false;
      }
      if (initObj.hasOwnProperty('laser_right_hit')) {
        this.laser_right_hit = initObj.laser_right_hit
      }
      else {
        this.laser_right_hit = false;
      }
      if (initObj.hasOwnProperty('depth_bottom_switch')) {
        this.depth_bottom_switch = initObj.depth_bottom_switch
      }
      else {
        this.depth_bottom_switch = false;
      }
      if (initObj.hasOwnProperty('depth_top_switch')) {
        this.depth_top_switch = initObj.depth_top_switch
      }
      else {
        this.depth_top_switch = false;
      }
      if (initObj.hasOwnProperty('yaw')) {
        this.yaw = initObj.yaw
      }
      else {
        this.yaw = 0.0;
      }
      if (initObj.hasOwnProperty('mass')) {
        this.mass = initObj.mass
      }
      else {
        this.mass = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type sensor_msg
    // Serialize message field [laser_top_hit]
    bufferOffset = _serializer.bool(obj.laser_top_hit, buffer, bufferOffset);
    // Serialize message field [laser_left_hit]
    bufferOffset = _serializer.bool(obj.laser_left_hit, buffer, bufferOffset);
    // Serialize message field [laser_right_hit]
    bufferOffset = _serializer.bool(obj.laser_right_hit, buffer, bufferOffset);
    // Serialize message field [depth_bottom_switch]
    bufferOffset = _serializer.bool(obj.depth_bottom_switch, buffer, bufferOffset);
    // Serialize message field [depth_top_switch]
    bufferOffset = _serializer.bool(obj.depth_top_switch, buffer, bufferOffset);
    // Serialize message field [yaw]
    bufferOffset = _serializer.float32(obj.yaw, buffer, bufferOffset);
    // Serialize message field [mass]
    bufferOffset = _serializer.float32(obj.mass, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type sensor_msg
    let len;
    let data = new sensor_msg(null);
    // Deserialize message field [laser_top_hit]
    data.laser_top_hit = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [laser_left_hit]
    data.laser_left_hit = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [laser_right_hit]
    data.laser_right_hit = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [depth_bottom_switch]
    data.depth_bottom_switch = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [depth_top_switch]
    data.depth_top_switch = _deserializer.bool(buffer, bufferOffset);
    // Deserialize message field [yaw]
    data.yaw = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [mass]
    data.mass = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 13;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mars_robot_msgs/sensor_msg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '57d6b1ca27430172008d546be1f39dfb';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    bool laser_top_hit
    bool laser_left_hit
    bool laser_right_hit
    bool depth_bottom_switch
    bool depth_top_switch
    float32 yaw
    float32 mass
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new sensor_msg(null);
    if (msg.laser_top_hit !== undefined) {
      resolved.laser_top_hit = msg.laser_top_hit;
    }
    else {
      resolved.laser_top_hit = false
    }

    if (msg.laser_left_hit !== undefined) {
      resolved.laser_left_hit = msg.laser_left_hit;
    }
    else {
      resolved.laser_left_hit = false
    }

    if (msg.laser_right_hit !== undefined) {
      resolved.laser_right_hit = msg.laser_right_hit;
    }
    else {
      resolved.laser_right_hit = false
    }

    if (msg.depth_bottom_switch !== undefined) {
      resolved.depth_bottom_switch = msg.depth_bottom_switch;
    }
    else {
      resolved.depth_bottom_switch = false
    }

    if (msg.depth_top_switch !== undefined) {
      resolved.depth_top_switch = msg.depth_top_switch;
    }
    else {
      resolved.depth_top_switch = false
    }

    if (msg.yaw !== undefined) {
      resolved.yaw = msg.yaw;
    }
    else {
      resolved.yaw = 0.0
    }

    if (msg.mass !== undefined) {
      resolved.mass = msg.mass;
    }
    else {
      resolved.mass = 0.0
    }

    return resolved;
    }
};

module.exports = sensor_msg;
