// Auto-generated. Do not edit!

// (in-package panthera_locomotion.msg)


"use strict";

const _serializer = _ros_msg_utils.Serialize;
const _arraySerializer = _serializer.Array;
const _deserializer = _ros_msg_utils.Deserialize;
const _arrayDeserializer = _deserializer.Array;
const _finder = _ros_msg_utils.Find;
const _getByteLength = _ros_msg_utils.getByteLength;

//-----------------------------------------------------------

class Custom_msg {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.current = null;
      this.speed = null;
      this.target_speed = null;
      this.pid_speed = null;
    }
    else {
      if (initObj.hasOwnProperty('current')) {
        this.current = initObj.current
      }
      else {
        this.current = 0.0;
      }
      if (initObj.hasOwnProperty('speed')) {
        this.speed = initObj.speed
      }
      else {
        this.speed = 0.0;
      }
      if (initObj.hasOwnProperty('target_speed')) {
        this.target_speed = initObj.target_speed
      }
      else {
        this.target_speed = 0.0;
      }
      if (initObj.hasOwnProperty('pid_speed')) {
        this.pid_speed = initObj.pid_speed
      }
      else {
        this.pid_speed = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Custom_msg
    // Serialize message field [current]
    bufferOffset = _serializer.float64(obj.current, buffer, bufferOffset);
    // Serialize message field [speed]
    bufferOffset = _serializer.float64(obj.speed, buffer, bufferOffset);
    // Serialize message field [target_speed]
    bufferOffset = _serializer.float64(obj.target_speed, buffer, bufferOffset);
    // Serialize message field [pid_speed]
    bufferOffset = _serializer.float64(obj.pid_speed, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Custom_msg
    let len;
    let data = new Custom_msg(null);
    // Deserialize message field [current]
    data.current = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [speed]
    data.speed = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [target_speed]
    data.target_speed = _deserializer.float64(buffer, bufferOffset);
    // Deserialize message field [pid_speed]
    data.pid_speed = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 32;
  }

  static datatype() {
    // Returns string type for a message object
    return 'panthera_locomotion/Custom_msg';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '70d45acb8d6001240e5e0b4d267d7330';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 current
    float64 speed
    float64 target_speed
    float64 pid_speed
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Custom_msg(null);
    if (msg.current !== undefined) {
      resolved.current = msg.current;
    }
    else {
      resolved.current = 0.0
    }

    if (msg.speed !== undefined) {
      resolved.speed = msg.speed;
    }
    else {
      resolved.speed = 0.0
    }

    if (msg.target_speed !== undefined) {
      resolved.target_speed = msg.target_speed;
    }
    else {
      resolved.target_speed = 0.0
    }

    if (msg.pid_speed !== undefined) {
      resolved.pid_speed = msg.pid_speed;
    }
    else {
      resolved.pid_speed = 0.0
    }

    return resolved;
    }
};

module.exports = Custom_msg;
