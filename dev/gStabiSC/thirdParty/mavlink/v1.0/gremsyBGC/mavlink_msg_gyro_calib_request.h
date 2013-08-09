// MESSAGE GYRO_CALIB_REQUEST PACKING

#define MAVLINK_MSG_ID_GYRO_CALIB_REQUEST 156

typedef struct __mavlink_gyro_calib_request_t
{
 uint8_t gyro_calib_mode; ///< gyro calib mode
} mavlink_gyro_calib_request_t;

#define MAVLINK_MSG_ID_GYRO_CALIB_REQUEST_LEN 1
#define MAVLINK_MSG_ID_156_LEN 1



#define MAVLINK_MESSAGE_INFO_GYRO_CALIB_REQUEST { \
	"GYRO_CALIB_REQUEST", \
	1, \
	{  { "gyro_calib_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_gyro_calib_request_t, gyro_calib_mode) }, \
         } \
}


/**
 * @brief Pack a gyro_calib_request message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param gyro_calib_mode gyro calib mode
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gyro_calib_request_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t gyro_calib_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[1];
	_mav_put_uint8_t(buf, 0, gyro_calib_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 1);
#else
	mavlink_gyro_calib_request_t packet;
	packet.gyro_calib_mode = gyro_calib_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 1);
#endif

	msg->msgid = MAVLINK_MSG_ID_GYRO_CALIB_REQUEST;
	return mavlink_finalize_message(msg, system_id, component_id, 1, 231);
}

/**
 * @brief Pack a gyro_calib_request message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param gyro_calib_mode gyro calib mode
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gyro_calib_request_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t gyro_calib_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[1];
	_mav_put_uint8_t(buf, 0, gyro_calib_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 1);
#else
	mavlink_gyro_calib_request_t packet;
	packet.gyro_calib_mode = gyro_calib_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 1);
#endif

	msg->msgid = MAVLINK_MSG_ID_GYRO_CALIB_REQUEST;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 1, 231);
}

/**
 * @brief Encode a gyro_calib_request struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gyro_calib_request C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gyro_calib_request_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gyro_calib_request_t* gyro_calib_request)
{
	return mavlink_msg_gyro_calib_request_pack(system_id, component_id, msg, gyro_calib_request->gyro_calib_mode);
}

/**
 * @brief Send a gyro_calib_request message
 * @param chan MAVLink channel to send the message
 *
 * @param gyro_calib_mode gyro calib mode
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gyro_calib_request_send(mavlink_channel_t chan, uint8_t gyro_calib_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[1];
	_mav_put_uint8_t(buf, 0, gyro_calib_mode);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GYRO_CALIB_REQUEST, buf, 1, 231);
#else
	mavlink_gyro_calib_request_t packet;
	packet.gyro_calib_mode = gyro_calib_mode;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GYRO_CALIB_REQUEST, (const char *)&packet, 1, 231);
#endif
}

#endif

// MESSAGE GYRO_CALIB_REQUEST UNPACKING


/**
 * @brief Get field gyro_calib_mode from gyro_calib_request message
 *
 * @return gyro calib mode
 */
static inline uint8_t mavlink_msg_gyro_calib_request_get_gyro_calib_mode(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a gyro_calib_request message into a struct
 *
 * @param msg The message to decode
 * @param gyro_calib_request C-struct to decode the message contents into
 */
static inline void mavlink_msg_gyro_calib_request_decode(const mavlink_message_t* msg, mavlink_gyro_calib_request_t* gyro_calib_request)
{
#if MAVLINK_NEED_BYTE_SWAP
	gyro_calib_request->gyro_calib_mode = mavlink_msg_gyro_calib_request_get_gyro_calib_mode(msg);
#else
	memcpy(gyro_calib_request, _MAV_PAYLOAD(msg), 1);
#endif
}
