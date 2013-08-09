// MESSAGE GYRO_CALIB_STATUS PACKING

#define MAVLINK_MSG_ID_GYRO_CALIB_STATUS 157

typedef struct __mavlink_gyro_calib_status_t
{
 uint8_t status; ///< gyro calib status
} mavlink_gyro_calib_status_t;

#define MAVLINK_MSG_ID_GYRO_CALIB_STATUS_LEN 1
#define MAVLINK_MSG_ID_157_LEN 1



#define MAVLINK_MESSAGE_INFO_GYRO_CALIB_STATUS { \
	"GYRO_CALIB_STATUS", \
	1, \
	{  { "status", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_gyro_calib_status_t, status) }, \
         } \
}


/**
 * @brief Pack a gyro_calib_status message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param status gyro calib status
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gyro_calib_status_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[1];
	_mav_put_uint8_t(buf, 0, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 1);
#else
	mavlink_gyro_calib_status_t packet;
	packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 1);
#endif

	msg->msgid = MAVLINK_MSG_ID_GYRO_CALIB_STATUS;
	return mavlink_finalize_message(msg, system_id, component_id, 1, 196);
}

/**
 * @brief Pack a gyro_calib_status message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param status gyro calib status
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gyro_calib_status_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[1];
	_mav_put_uint8_t(buf, 0, status);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 1);
#else
	mavlink_gyro_calib_status_t packet;
	packet.status = status;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 1);
#endif

	msg->msgid = MAVLINK_MSG_ID_GYRO_CALIB_STATUS;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 1, 196);
}

/**
 * @brief Encode a gyro_calib_status struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param gyro_calib_status C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_gyro_calib_status_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_gyro_calib_status_t* gyro_calib_status)
{
	return mavlink_msg_gyro_calib_status_pack(system_id, component_id, msg, gyro_calib_status->status);
}

/**
 * @brief Send a gyro_calib_status message
 * @param chan MAVLink channel to send the message
 *
 * @param status gyro calib status
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gyro_calib_status_send(mavlink_channel_t chan, uint8_t status)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[1];
	_mav_put_uint8_t(buf, 0, status);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GYRO_CALIB_STATUS, buf, 1, 196);
#else
	mavlink_gyro_calib_status_t packet;
	packet.status = status;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GYRO_CALIB_STATUS, (const char *)&packet, 1, 196);
#endif
}

#endif

// MESSAGE GYRO_CALIB_STATUS UNPACKING


/**
 * @brief Get field status from gyro_calib_status message
 *
 * @return gyro calib status
 */
static inline uint8_t mavlink_msg_gyro_calib_status_get_status(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a gyro_calib_status message into a struct
 *
 * @param msg The message to decode
 * @param gyro_calib_status C-struct to decode the message contents into
 */
static inline void mavlink_msg_gyro_calib_status_decode(const mavlink_message_t* msg, mavlink_gyro_calib_status_t* gyro_calib_status)
{
#if MAVLINK_NEED_BYTE_SWAP
	gyro_calib_status->status = mavlink_msg_gyro_calib_status_get_status(msg);
#else
	memcpy(gyro_calib_status, _MAV_PAYLOAD(msg), 1);
#endif
}
