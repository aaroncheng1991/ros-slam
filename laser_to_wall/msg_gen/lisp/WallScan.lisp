; Auto-generated. Do not edit!


(cl:in-package laser_to_wall-msg)


;//! \htmlinclude WallScan.msg.html

(cl:defclass <WallScan> (roslisp-msg-protocol:ros-message)
  ((header
    :reader header
    :initarg :header
    :type std_msgs-msg:Header
    :initform (cl:make-instance 'std_msgs-msg:Header))
   (wall_left
    :reader wall_left
    :initarg :wall_left
    :type cl:boolean
    :initform cl:nil)
   (wall_right
    :reader wall_right
    :initarg :wall_right
    :type cl:boolean
    :initform cl:nil)
   (wall_front
    :reader wall_front
    :initarg :wall_front
    :type cl:boolean
    :initform cl:nil))
)

(cl:defclass WallScan (<WallScan>)
  ())

(cl:defmethod cl:initialize-instance :after ((m <WallScan>) cl:&rest args)
  (cl:declare (cl:ignorable args))
  (cl:unless (cl:typep m 'WallScan)
    (roslisp-msg-protocol:msg-deprecation-warning "using old message class name laser_to_wall-msg:<WallScan> is deprecated: use laser_to_wall-msg:WallScan instead.")))

(cl:ensure-generic-function 'header-val :lambda-list '(m))
(cl:defmethod header-val ((m <WallScan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader laser_to_wall-msg:header-val is deprecated.  Use laser_to_wall-msg:header instead.")
  (header m))

(cl:ensure-generic-function 'wall_left-val :lambda-list '(m))
(cl:defmethod wall_left-val ((m <WallScan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader laser_to_wall-msg:wall_left-val is deprecated.  Use laser_to_wall-msg:wall_left instead.")
  (wall_left m))

(cl:ensure-generic-function 'wall_right-val :lambda-list '(m))
(cl:defmethod wall_right-val ((m <WallScan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader laser_to_wall-msg:wall_right-val is deprecated.  Use laser_to_wall-msg:wall_right instead.")
  (wall_right m))

(cl:ensure-generic-function 'wall_front-val :lambda-list '(m))
(cl:defmethod wall_front-val ((m <WallScan>))
  (roslisp-msg-protocol:msg-deprecation-warning "Using old-style slot reader laser_to_wall-msg:wall_front-val is deprecated.  Use laser_to_wall-msg:wall_front instead.")
  (wall_front m))
(cl:defmethod roslisp-msg-protocol:serialize ((msg <WallScan>) ostream)
  "Serializes a message object of type '<WallScan>"
  (roslisp-msg-protocol:serialize (cl:slot-value msg 'header) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'wall_left) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'wall_right) 1 0)) ostream)
  (cl:write-byte (cl:ldb (cl:byte 8 0) (cl:if (cl:slot-value msg 'wall_front) 1 0)) ostream)
)
(cl:defmethod roslisp-msg-protocol:deserialize ((msg <WallScan>) istream)
  "Deserializes a message object of type '<WallScan>"
  (roslisp-msg-protocol:deserialize (cl:slot-value msg 'header) istream)
    (cl:setf (cl:slot-value msg 'wall_left) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'wall_right) (cl:not (cl:zerop (cl:read-byte istream))))
    (cl:setf (cl:slot-value msg 'wall_front) (cl:not (cl:zerop (cl:read-byte istream))))
  msg
)
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql '<WallScan>)))
  "Returns string type for a message object of type '<WallScan>"
  "laser_to_wall/WallScan")
(cl:defmethod roslisp-msg-protocol:ros-datatype ((msg (cl:eql 'WallScan)))
  "Returns string type for a message object of type 'WallScan"
  "laser_to_wall/WallScan")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql '<WallScan>)))
  "Returns md5sum for a message object of type '<WallScan>"
  "b3eac0369e6437026a45b36a926ada13")
(cl:defmethod roslisp-msg-protocol:md5sum ((type (cl:eql 'WallScan)))
  "Returns md5sum for a message object of type 'WallScan"
  "b3eac0369e6437026a45b36a926ada13")
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql '<WallScan>)))
  "Returns full string definition for message of type '<WallScan>"
  (cl:format cl:nil "Header header~%bool wall_left~%bool wall_right~%bool wall_front~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:message-definition ((type (cl:eql 'WallScan)))
  "Returns full string definition for message of type 'WallScan"
  (cl:format cl:nil "Header header~%bool wall_left~%bool wall_right~%bool wall_front~%================================================================================~%MSG: std_msgs/Header~%# Standard metadata for higher-level stamped data types.~%# This is generally used to communicate timestamped data ~%# in a particular coordinate frame.~%# ~%# sequence ID: consecutively increasing ID ~%uint32 seq~%#Two-integer timestamp that is expressed as:~%# * stamp.secs: seconds (stamp_secs) since epoch~%# * stamp.nsecs: nanoseconds since stamp_secs~%# time-handling sugar is provided by the client library~%time stamp~%#Frame this data is associated with~%# 0: no frame~%# 1: global frame~%string frame_id~%~%~%"))
(cl:defmethod roslisp-msg-protocol:serialization-length ((msg <WallScan>))
  (cl:+ 0
     (roslisp-msg-protocol:serialization-length (cl:slot-value msg 'header))
     1
     1
     1
))
(cl:defmethod roslisp-msg-protocol:ros-message-to-list ((msg <WallScan>))
  "Converts a ROS message object to a list"
  (cl:list 'WallScan
    (cl:cons ':header (header msg))
    (cl:cons ':wall_left (wall_left msg))
    (cl:cons ':wall_right (wall_right msg))
    (cl:cons ':wall_front (wall_front msg))
))
