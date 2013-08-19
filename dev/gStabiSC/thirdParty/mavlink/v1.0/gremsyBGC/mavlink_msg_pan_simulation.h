// MESSAGE PAN_SIMULATION PACKING

#define MAVLINK_MSG_ID_PAN_SIMULATION 155

typedef struct __mavlink_pan_simulation_t
{
 int16_t pan; ///< pan value in degree
} mavlink_pan_simulation_t;

#define MAVLINK_MSG_ID_PAN_SIMULATION_LEN 2
#define MAVLINK_MSG_ID_155_LEN 2



#define MAVLINK_MESSAGE_INFO_PAN_SIMULATION { \
	"PAN_SIMULATION", \
	1, \
	{  { "pan", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_pan_simulation_t, pan) }, \
         } \
}


/**
 * @brief Pack a pan_simulation message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param pan pan value in degree
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pan_simulation_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int16_t pan)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[2];
	_mav_put_int16_t(buf, 0, pan);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 2);
#else
	mavlink_pan_simulation_t packet;
	packet.pan = pan;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 2);
#endif

	msg->msgid = MAVLINK_MSG_ID_PAN_SIMULATION;
	return mavlink_finalize_message(msg, system_id, component_id, 2, 145);
}

/**
 * @brief Pack a pan_simulation message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param pan pan value in degree
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_pan_simulation_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int16_t pan)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[2];
	_mav_put_int16_t(buf, 0, pan);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 2);
#else
	mavlink_pan_simulation_t packet;
	packet.pan = pan;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 2);
#endif

	msg->msgid = MAVLINK_MSG_ID_PAN_SIMULATION;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 2, 145);
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
	return mavlink_msg_pan_simulation_pack(system_id, component_id, msg, pan_simulation->pan);
}

/**
 * @brief Send a pan_simulation message
 * @param chan MAVLink channel to send the message
 *
 * @param pan pan value in degree
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_pan_simulation_send(mavlink_channel_t chan, int16_t pan)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[2];
	_mav_put_int16_t(buf, 0, pan);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PAN_SIMULATION, buf, 2, 145);
#else
	mavlink_pan_simulation_t packet;
	packet.pan = pan;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_PAN_SIMULATION, (const char *)&packet, 2, 145);
#endif
}

#endif

// MESSAGE PAN_SIMULATION UNPACKING


/**
 * @brief Get field pan from pan_simulation message
 *
 * @return pan value in degree
 */
static inline int16_t mavlink_msg_pan_simulation_get_pan(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  0);
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
#else
	memcpy(pan_simulation, _MAV_PAYLOAD(msg), 2);
#endif
}
