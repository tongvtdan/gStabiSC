// MESSAGE USE_RC_SIMULATOR PACKING

#define MAVLINK_MSG_ID_USE_RC_SIMULATOR 160

typedef struct __mavlink_use_rc_simulator_t
{
 uint8_t simulator_cmd; ///< simulator command on or off
} mavlink_use_rc_simulator_t;

#define MAVLINK_MSG_ID_USE_RC_SIMULATOR_LEN 1
#define MAVLINK_MSG_ID_160_LEN 1



#define MAVLINK_MESSAGE_INFO_USE_RC_SIMULATOR { \
	"USE_RC_SIMULATOR", \
	1, \
	{  { "simulator_cmd", NULL, MAVLINK_TYPE_UINT8_T, 0, 0, offsetof(mavlink_use_rc_simulator_t, simulator_cmd) }, \
         } \
}


/**
 * @brief Pack a use_rc_simulator message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param simulator_cmd simulator command on or off
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_use_rc_simulator_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       uint8_t simulator_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[1];
	_mav_put_uint8_t(buf, 0, simulator_cmd);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 1);
#else
	mavlink_use_rc_simulator_t packet;
	packet.simulator_cmd = simulator_cmd;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 1);
#endif

	msg->msgid = MAVLINK_MSG_ID_USE_RC_SIMULATOR;
	return mavlink_finalize_message(msg, system_id, component_id, 1, 17);
}

/**
 * @brief Pack a use_rc_simulator message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param simulator_cmd simulator command on or off
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_use_rc_simulator_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           uint8_t simulator_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[1];
	_mav_put_uint8_t(buf, 0, simulator_cmd);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 1);
#else
	mavlink_use_rc_simulator_t packet;
	packet.simulator_cmd = simulator_cmd;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 1);
#endif

	msg->msgid = MAVLINK_MSG_ID_USE_RC_SIMULATOR;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 1, 17);
}

/**
 * @brief Encode a use_rc_simulator struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param use_rc_simulator C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_use_rc_simulator_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_use_rc_simulator_t* use_rc_simulator)
{
	return mavlink_msg_use_rc_simulator_pack(system_id, component_id, msg, use_rc_simulator->simulator_cmd);
}

/**
 * @brief Send a use_rc_simulator message
 * @param chan MAVLink channel to send the message
 *
 * @param simulator_cmd simulator command on or off
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_use_rc_simulator_send(mavlink_channel_t chan, uint8_t simulator_cmd)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[1];
	_mav_put_uint8_t(buf, 0, simulator_cmd);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_USE_RC_SIMULATOR, buf, 1, 17);
#else
	mavlink_use_rc_simulator_t packet;
	packet.simulator_cmd = simulator_cmd;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_USE_RC_SIMULATOR, (const char *)&packet, 1, 17);
#endif
}

#endif

// MESSAGE USE_RC_SIMULATOR UNPACKING


/**
 * @brief Get field simulator_cmd from use_rc_simulator message
 *
 * @return simulator command on or off
 */
static inline uint8_t mavlink_msg_use_rc_simulator_get_simulator_cmd(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  0);
}

/**
 * @brief Decode a use_rc_simulator message into a struct
 *
 * @param msg The message to decode
 * @param use_rc_simulator C-struct to decode the message contents into
 */
static inline void mavlink_msg_use_rc_simulator_decode(const mavlink_message_t* msg, mavlink_use_rc_simulator_t* use_rc_simulator)
{
#if MAVLINK_NEED_BYTE_SWAP
	use_rc_simulator->simulator_cmd = mavlink_msg_use_rc_simulator_get_simulator_cmd(msg);
#else
	memcpy(use_rc_simulator, _MAV_PAYLOAD(msg), 1);
#endif
}
