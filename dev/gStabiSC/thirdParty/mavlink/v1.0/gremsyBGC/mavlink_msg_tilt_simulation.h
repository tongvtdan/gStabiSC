// MESSAGE TILT_SIMULATION PACKING

#define MAVLINK_MSG_ID_TILT_SIMULATION 153

typedef struct __mavlink_tilt_simulation_t
{
 int16_t tilt; ///< tilt value
 uint8_t channel; ///< tilt channel
} mavlink_tilt_simulation_t;

#define MAVLINK_MSG_ID_TILT_SIMULATION_LEN 3
#define MAVLINK_MSG_ID_153_LEN 3



#define MAVLINK_MESSAGE_INFO_TILT_SIMULATION { \
	"TILT_SIMULATION", \
	2, \
	{  { "tilt", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_tilt_simulation_t, tilt) }, \
         { "channel", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_tilt_simulation_t, channel) }, \
         } \
}


/**
 * @brief Pack a tilt_simulation message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param tilt tilt value
 * @param channel tilt channel
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tilt_simulation_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int16_t tilt, uint8_t channel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[3];
	_mav_put_int16_t(buf, 0, tilt);
	_mav_put_uint8_t(buf, 2, channel);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 3);
#else
	mavlink_tilt_simulation_t packet;
	packet.tilt = tilt;
	packet.channel = channel;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 3);
#endif

	msg->msgid = MAVLINK_MSG_ID_TILT_SIMULATION;
	return mavlink_finalize_message(msg, system_id, component_id, 3, 35);
}

/**
 * @brief Pack a tilt_simulation message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param tilt tilt value
 * @param channel tilt channel
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tilt_simulation_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int16_t tilt,uint8_t channel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[3];
	_mav_put_int16_t(buf, 0, tilt);
	_mav_put_uint8_t(buf, 2, channel);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 3);
#else
	mavlink_tilt_simulation_t packet;
	packet.tilt = tilt;
	packet.channel = channel;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 3);
#endif

	msg->msgid = MAVLINK_MSG_ID_TILT_SIMULATION;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 3, 35);
}

/**
 * @brief Encode a tilt_simulation struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param tilt_simulation C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_tilt_simulation_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_tilt_simulation_t* tilt_simulation)
{
	return mavlink_msg_tilt_simulation_pack(system_id, component_id, msg, tilt_simulation->tilt, tilt_simulation->channel);
}

/**
 * @brief Send a tilt_simulation message
 * @param chan MAVLink channel to send the message
 *
 * @param tilt tilt value
 * @param channel tilt channel
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_tilt_simulation_send(mavlink_channel_t chan, int16_t tilt, uint8_t channel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[3];
	_mav_put_int16_t(buf, 0, tilt);
	_mav_put_uint8_t(buf, 2, channel);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TILT_SIMULATION, buf, 3, 35);
#else
	mavlink_tilt_simulation_t packet;
	packet.tilt = tilt;
	packet.channel = channel;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TILT_SIMULATION, (const char *)&packet, 3, 35);
#endif
}

#endif

// MESSAGE TILT_SIMULATION UNPACKING


/**
 * @brief Get field tilt from tilt_simulation message
 *
 * @return tilt value
 */
static inline int16_t mavlink_msg_tilt_simulation_get_tilt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field channel from tilt_simulation message
 *
 * @return tilt channel
 */
static inline uint8_t mavlink_msg_tilt_simulation_get_channel(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a tilt_simulation message into a struct
 *
 * @param msg The message to decode
 * @param tilt_simulation C-struct to decode the message contents into
 */
static inline void mavlink_msg_tilt_simulation_decode(const mavlink_message_t* msg, mavlink_tilt_simulation_t* tilt_simulation)
{
#if MAVLINK_NEED_BYTE_SWAP
	tilt_simulation->tilt = mavlink_msg_tilt_simulation_get_tilt(msg);
	tilt_simulation->channel = mavlink_msg_tilt_simulation_get_channel(msg);
#else
	memcpy(tilt_simulation, _MAV_PAYLOAD(msg), 3);
#endif
}
