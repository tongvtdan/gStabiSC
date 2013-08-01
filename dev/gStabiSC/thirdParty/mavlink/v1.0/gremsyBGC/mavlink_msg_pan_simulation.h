// MESSAGE PAN_SIMULATION PACKING

#define MAVLINK_MSG_ID_PAN_SIMULATION 155

typedef struct __mavlink_pan_simulation_t
{
 int16_t pan; ///< pan value
 uint8_t channel; ///< pan channel
} mavlink_pan_simulation_t;

#define MAVLINK_MSG_ID_PAN_SIMULATION_LEN 3
#define MAVLINK_MSG_ID_155_LEN 3



#define MAVLINK_MESSAGE_INFO_PAN_SIMULATION { \
	"PAN_SIMULATION", \
	2, \
	{  { "pan", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_pan_simulation_t, pan) }, \
         { "channel", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_pan_simulation_t, channel) }, \
         } \
}


/**
 * @brief Pack a pan_simulation message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param pan pan value
 * @param channel pan channel
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pan_simulation_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int16_t pan, uint8_t channel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[3];
	_mav_put_int16_t(buf, 0, pan);
	_mav_put_uint8_t(buf, 2, channel);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 3);
#else
	mavlink_pan_simulation_t packet;
	packet.pan = pan;
	packet.channel = channel;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 3);
#endif

	msg->msgid = MAVLINK_MSG_ID_PAN_SIMULATION;
	return mavlink_finalize_message(msg, system_id, component_id, 3, 192);
}

/**
 * @brief Pack a pan_simulation message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param pan pan value
 * @param channel pan channel
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pan_simulation_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int16_t pan,uint8_t channel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[3];
	_mav_put_int16_t(buf, 0, pan);
	_mav_put_uint8_t(buf, 2, channel);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 3);
#else
	mavlink_pan_simulation_t packet;
	packet.pan = pan;
	packet.channel = channel;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 3);
#endif

	msg->msgid = MAVLINK_MSG_ID_PAN_SIMULATION;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 3, 192);
}

/**
 * @brief Encode a pan_simulation struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param pan_simulation C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_pan_simulation_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_pan_simulation_t* pan_simulation)
{
	return mavlink_msg_pan_simulation_pack(system_id, component_id, msg, pan_simulation->pan, pan_simulation->channel);
}

/**
 * @brief Send a pan_simulation message
 * @param chan MAVLink channel to send the message
 *
 * @param pan pan value
 * @param channel pan channel
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_pan_simulation_send(mavlink_channel_t chan, int16_t pan, uint8_t channel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[3];
	_mav_put_int16_t(buf, 0, pan);
	_mav_put_uint8_t(buf, 2, channel);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PAN_SIMULATION, buf, 3, 192);
#else
	mavlink_pan_simulation_t packet;
	packet.pan = pan;
	packet.channel = channel;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PAN_SIMULATION, (const char *)&packet, 3, 192);
#endif
}

#endif

// MESSAGE PAN_SIMULATION UNPACKING


/**
 * @brief Get field pan from pan_simulation message
 *
 * @return pan value
 */
static inline int16_t mavlink_msg_pan_simulation_get_pan(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field channel from pan_simulation message
 *
 * @return pan channel
 */
static inline uint8_t mavlink_msg_pan_simulation_get_channel(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a pan_simulation message into a struct
 *
 * @param msg The message to decode
 * @param pan_simulation C-struct to decode the message contents into
 */
static inline void mavlink_msg_pan_simulation_decode(const mavlink_message_t* msg, mavlink_pan_simulation_t* pan_simulation)
{
#if MAVLINK_NEED_BYTE_SWAP
	pan_simulation->pan = mavlink_msg_pan_simulation_get_pan(msg);
	pan_simulation->channel = mavlink_msg_pan_simulation_get_channel(msg);
#else
	memcpy(pan_simulation, _MAV_PAYLOAD(msg), 3);
#endif
}
