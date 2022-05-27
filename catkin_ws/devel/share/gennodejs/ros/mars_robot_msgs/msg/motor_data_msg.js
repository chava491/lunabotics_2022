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

class motor_data_msg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.auger_current = null;
      this.auger_speed = null;
      this.right_loco_current = null;
      this.left_loco_current = null;
    }
    else {
      if (initObj.hasOwnProperty('auger_current')) {
        this.auger_current = initObj.auger_current
      }
      else {
        this.auger_current = 0.0;
      }
      if (initObj.hasOwnProperty('auger_speed')) {
        this.auger_speed = initObj.auger_speed
      }
      else {
        this.auger_speed = 0.0;
      }
      if (initObj.hasOwnProperty('right_loco_current')) {
        this.right_loco_current = initObj.right_loco_current
      }
      else {
        this.right_loco_current = 0.0;
      }
      if (initObj.hasOwnProperty('left_loco_current')) {
        this.left_loco_current = initObj.left_loco_current
      }
      else {
        this.left_loco_current = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type motor_data_msg
    // Serialize message field [auger_current]
    bufferOffset = _serializer.float32(obj.auger_current, buffer, bufferOffset);
    // Serialize message field [auger_speed]
    bufferOffset = _serializer.float32(obj.auger_speed, buffer, bufferOffset);
    // Serialize message field [right_loco_current]
    bufferOffset = _serializer.float32(obj.right_loco_current, buffer, bufferOffset);
    // Serialize message field [left_loco_current]
    bufferOffset = _serializer.float32(obj.left_loco_current, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type motor_data_msg
    let len;
    let data = new motor_data_msg(null);
    // Deserialize message field [auger_current]
    data.auger_current = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [auger_speed]
    data.auger_speed = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [right_loco_current]
    data.right_loco_current = _deserializer.float32(buffer, bufferOffset);
    // Deserialize message field [left_loco_current]
    data.left_loco_current = _deserializer.float32(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 16;
  }

  static datatype() {
    // Returns string type for a message object
    return 'mars_robot_msgs/motor_data_msg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '1a39659befe0182ae29a42de47aa67fc';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float32 auger_current
    float32 auger_speed
    float32 right_loco_current
    float32 left_loco_current
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new motor_data_msg(null);
    if (msg.auger_current !== undefined) {
      resolved.auger_current = msg.auger_current;
    }
    else {
      resolved.auger_current = 0.0
    }

    if (msg.auger_speed !== undefined) {
      resolved.auger_speed = msg.auger_speed;
    }
    else {
      resolved.auger_speed = 0.0
    }

    if (msg.right_loco_current !== undefined) {
      resolved.right_loco_current = msg.right_loco_current;
    }
    else {
      resolved.right_loco_current = 0.0
    }

    if (msg.left_loco_current !== undefined) {
      resolved.left_loco_current = msg.left_loco_current;
    }
    else {
      resolved.left_loco_current = 0.0
    }

    return resolved;
    }
};

module.exports = motor_data_msg;
