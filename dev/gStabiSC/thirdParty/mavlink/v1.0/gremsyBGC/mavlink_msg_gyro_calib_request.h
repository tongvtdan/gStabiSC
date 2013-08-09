// MESSAGE GYRO_CALIB_REQUEST PACKING

#define MAVLINK_MSG_ID_GYRO_CALIB_REQUEST 156

typedef struct __mavlink_gyro_calib_request_t
{

} mavlink_gyro_calib_request_t;

#define MAVLINK_MSG_ID_GYRO_CALIB_REQUEST_LEN 0
#define MAVLINK_MSG_ID_156_LEN 0



#define MAVLINK_MESSAGE_INFO_GYRO_CALIB_REQUEST { \
	"GYRO_CALIB_REQUEST", \
	0, \
	{  } \
}


/**
 * @brief Pack a gyro_calib_request message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *

 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gyro_calib_request_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						      )
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[0];


        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 0);
#else
	mavlink_gyro_calib_request_t packet;


        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 0);
#endif

	msg->msgid = MAVLINK_MSG_ID_GYRO_CALIB_REQUEST;
	return mavlink_finalize_message(msg, system_id, component_id, 0, 205);
}

/**
 * @brief Pack a gyro_calib_request message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into

 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_gyro_calib_request_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           )
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[0];


        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 0);
#else
	mavlink_gyro_calib_request_t packet;


        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 0);
#endif

	msg->msgid = MAVLINK_MSG_ID_GYRO_CALIB_REQUEST;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 0, 205);
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
	return mavlink_msg_gyro_calib_request_pack(system_id, component_id, msg,);
}

/**
 * @brief Send a gyro_calib_request message
 * @param chan MAVLink channel to send the message
 *

 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_gyro_calib_request_send(mavlink_channel_t chan,)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[0];


	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GYRO_CALIB_REQUEST, buf, 0, 205);
#else
	mavlink_gyro_calib_request_t packet;


	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_GYRO_CALIB_REQUEST, (const char *)&packet, 0, 205);
#endif
}

#endif

// MESSAGE GYRO_CALIB_REQUEST UNPACKING



/**
 * @brief Decode a gyro_calib_request message into a struct
 *
 * @param msg The message to decode
 * @param gyro_calib_request C-struct to decode the message contents into
 */
static inline void mavlink_msg_gyro_calib_request_decode(const mavlink_message_t* msg, mavlink_gyro_calib_request_t* gyro_calib_request)
{
#if MAVLINK_NEED_BYTE_SWAP

#else
	memcpy(gyro_calib_request, _MAV_PAYLOAD(msg), 0);
#endif
}
