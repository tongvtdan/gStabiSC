// MESSAGE ACC_CALIB_REQUEST PACKING

#define MAVLINK_MSG_ID_ACC_CALIB_REQUEST 158

typedef struct __mavlink_acc_calib_request_t
{
 uint8_t acc_calib_mode; ///< acc calib mode "one face" or "six faces"
} mavlink_acc_calib_request_t;

#define MAVLINK_MSG_ID_ACC_CALIB_REQUEST_LEN 1
#define MAVLINK_MSG_ID_158_LEN 1



#define MAVLINK_MESSAGE_INFO_ACC_CALIB_REQUEST { \
	"ACC_CALIB_REQUEST", \
	1, \
	{  { "acc_calib_mode", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_acc_calib_request_t, acc_calib_mode) }, \
         } \
}


/**
 * @brief Pack a acc_calib_request message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param acc_calib_mode acc calib mode "one face" or "six faces"
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_acc_calib_request_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t acc_calib_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[1];
	_mav_put_uint8_t(buf, 0, acc_calib_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 1);
#else
	mavlink_acc_calib_request_t packet;
	packet.acc_calib_mode = acc_calib_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 1);
#endif

	msg->msgid = MAVLINK_MSG_ID_ACC_CALIB_REQUEST;
	return mavlink_finalize_message(msg, system_id, component_id, 1, 26);
}

/**
 * @brief Pack a acc_calib_request message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param acc_calib_mode acc calib mode "one face" or "six faces"
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_acc_calib_request_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t acc_calib_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[1];
	_mav_put_uint8_t(buf, 0, acc_calib_mode);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 1);
#else
	mavlink_acc_calib_request_t packet;
	packet.acc_calib_mode = acc_calib_mode;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 1);
#endif

	msg->msgid = MAVLINK_MSG_ID_ACC_CALIB_REQUEST;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 1, 26);
}

/**
 * @brief Encode a acc_calib_request struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param acc_calib_request C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_acc_calib_request_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_acc_calib_request_t* acc_calib_request)
{
	return mavlink_msg_acc_calib_request_pack(system_id, component_id, msg, acc_calib_request->acc_calib_mode);
}

/**
 * @brief Send a acc_calib_request message
 * @param chan MAVLink channel to send the message
 *
 * @param acc_calib_mode acc calib mode "one face" or "six faces"
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_acc_calib_request_send(mavlink_channel_t chan, uint8_t acc_calib_mode)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[1];
	_mav_put_uint8_t(buf, 0, acc_calib_mode);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACC_CALIB_REQUEST, buf, 1, 26);
#else
	mavlink_acc_calib_request_t packet;
	packet.acc_calib_mode = acc_calib_mode;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ACC_CALIB_REQUEST, (const char *)&packet, 1, 26);
#endif
}

#endif

// MESSAGE ACC_CALIB_REQUEST UNPACKING


/**
 * @brief Get field acc_calib_mode from acc_calib_request message
 *
 * @return acc calib mode "one face" or "six faces"
 */
static inline uint8_t mavlink_msg_acc_calib_request_get_acc_calib_mode(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a acc_calib_request message into a struct
 *
 * @param msg The message to decode
 * @param acc_calib_request C-struct to decode the message contents into
 */
static inline void mavlink_msg_acc_calib_request_decode(const mavlink_message_t* msg, mavlink_acc_calib_request_t* acc_calib_request)
{
#if MAVLINK_NEED_BYTE_SWAP
	acc_calib_request->acc_calib_mode = mavlink_msg_acc_calib_request_get_acc_calib_mode(msg);
#else
	memcpy(acc_calib_request, _MAV_PAYLOAD(msg), 1);
#endif
}
