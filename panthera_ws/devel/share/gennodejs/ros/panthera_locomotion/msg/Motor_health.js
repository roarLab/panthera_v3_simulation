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

class Motor_health {
  constructor(initObj={}) {
    if (initObj === null) {
      // initObj === null is a special case for deserialization where we don't initialize fields
      this.fuse_condition = null;
    }
    else {
      if (initObj.hasOwnProperty('fuse_condition')) {
        this.fuse_condition = initObj.fuse_condition
      }
      else {
        this.fuse_condition = 0.0;
      }
    }
  }

  static serialize(obj, buffer, bufferOffset) {
    // Serializes a message object of type Motor_health
    // Serialize message field [fuse_condition]
    bufferOffset = _serializer.float64(obj.fuse_condition, buffer, bufferOffset);
    return bufferOffset;
  }

  static deserialize(buffer, bufferOffset=[0]) {
    //deserializes a message object of type Motor_health
    let len;
    let data = new Motor_health(null);
    // Deserialize message field [fuse_condition]
    data.fuse_condition = _deserializer.float64(buffer, bufferOffset);
    return data;
  }

  static getMessageSize(object) {
    return 8;
  }

  static datatype() {
    // Returns string type for a message object
    return 'panthera_locomotion/Motor_health';
  }

  static md5sum() {
    //Returns md5sum for a message object
    return '4b811d7770caee98dbe252ed8c9b8311';
  }

  static messageDefinition() {
    // Returns full string definition for message
    return `
    float64 fuse_condition
    
    `;
  }

  static Resolve(msg) {
    // deep-construct a valid message object instance of whatever was passed in
    if (typeof msg !== 'object' || msg === null) {
      msg = {};
    }
    const resolved = new Motor_health(null);
    if (msg.fuse_condition !== undefined) {
      resolved.fuse_condition = msg.fuse_condition;
    }
    else {
      resolved.fuse_condition = 0.0
    }

    return resolved;
    }
};

module.exports = Motor_health;
