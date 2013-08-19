// MESSAGE TILT_SIMULATION PACKING

#define MAVLINK_MSG_ID_TILT_SIMULATION 153

typedef struct __mavlink_tilt_simulation_t
{
 int16_t tilt; ///< tilt value in degree
} mavlink_tilt_simulation_t;

#define MAVLINK_MSG_ID_TILT_SIMULATION_LEN 2
#define MAVLINK_MSG_ID_153_LEN 2



#define MAVLINK_MESSAGE_INFO_TILT_SIMULATION { \
	"TILT_SIMULATION", \
	1, \
	{  { "tilt", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_tilt_simulation_t, tilt) }, \
         } \
}


/**
 * @brief Pack a tilt_simulation message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param tilt tilt value in degree
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tilt_simulation_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int16_t tilt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[2];
	_mav_put_int16_t(buf, 0, tilt);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 2);
#else
	mavlink_tilt_simulation_t packet;
	packet.tilt = tilt;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 2);
#endif

	msg->msgid = MAVLINK_MSG_ID_TILT_SIMULATION;
	return mavlink_finalize_message(msg, system_id, component_id, 2, 71);
}

/**
 * @brief Pack a tilt_simulation message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param tilt tilt value in degree
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_tilt_simulation_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int16_t tilt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[2];
	_mav_put_int16_t(buf, 0, tilt);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 2);
#else
	mavlink_tilt_simulation_t packet;
	packet.tilt = tilt;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 2);
#endif

	msg->msgid = MAVLINK_MSG_ID_TILT_SIMULATION;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 2, 71);
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
	return mavlink_msg_tilt_simulation_pack(system_id, component_id, msg, tilt_simulation->tilt);
}

/**
 * @brief Send a tilt_simulation message
 * @param chan MAVLink channel to send the message
 *
 * @param tilt tilt value in degree
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_tilt_simulation_send(mavlink_channel_t chan, int16_t tilt)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[2];
	_mav_put_int16_t(buf, 0, tilt);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TILT_SIMULATION, buf, 2, 71);
#else
	mavlink_tilt_simulation_t packet;
	packet.tilt = tilt;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_TILT_SIMULATION, (const char *)&packet, 2, 71);
#endif
}

#endif

// MESSAGE TILT_SIMULATION UNPACKING


/**
 * @brief Get field tilt from tilt_simulation message
 *
 * @return tilt value in degree
 */
static inline int16_t mavlink_msg_tilt_simulation_get_tilt(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  0);
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
#else
	memcpy(tilt_simulation, _MAV_PAYLOAD(msg), 2);
#endif
}
