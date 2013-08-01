// MESSAGE ROLL_SIMULATION PACKING

#define MAVLINK_MSG_ID_ROLL_SIMULATION 154

typedef struct __mavlink_roll_simulation_t
{
 int16_t roll; ///< roll value
 uint8_t channel; ///< roll channel
} mavlink_roll_simulation_t;

#define MAVLINK_MSG_ID_ROLL_SIMULATION_LEN 3
#define MAVLINK_MSG_ID_154_LEN 3



#define MAVLINK_MESSAGE_INFO_ROLL_SIMULATION { \
	"ROLL_SIMULATION", \
	2, \
	{  { "roll", NULL, MAVLINK_TYPE_INT16_T, 0, 0, offsetof(mavlink_roll_simulation_t, roll) }, \
         { "channel", NULL, MAVLINK_TYPE_UINT8_T, 0, 2, offsetof(mavlink_roll_simulation_t, channel) }, \
         } \
}


/**
 * @brief Pack a roll_simulation message
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 *
 * @param roll roll value
 * @param channel roll channel
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_roll_simulation_pack(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg,
						       int16_t roll, uint8_t channel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[3];
	_mav_put_int16_t(buf, 0, roll);
	_mav_put_uint8_t(buf, 2, channel);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 3);
#else
	mavlink_roll_simulation_t packet;
	packet.roll = roll;
	packet.channel = channel;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 3);
#endif

	msg->msgid = MAVLINK_MSG_ID_ROLL_SIMULATION;
	return mavlink_finalize_message(msg, system_id, component_id, 3, 186);
}

/**
 * @brief Pack a roll_simulation message on a channel
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param chan The MAVLink channel this message was sent over
 * @param msg The MAVLink message to compress the data into
 * @param roll roll value
 * @param channel roll channel
 * @return length of the message in bytes (excluding serial stream start sign)
 */
static inline uint16_t mavlink_msg_roll_simulation_pack_chan(uint8_t system_id, uint8_t component_id, uint8_t chan,
							   mavlink_message_t* msg,
						           int16_t roll,uint8_t channel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[3];
	_mav_put_int16_t(buf, 0, roll);
	_mav_put_uint8_t(buf, 2, channel);

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), buf, 3);
#else
	mavlink_roll_simulation_t packet;
	packet.roll = roll;
	packet.channel = channel;

        memcpy(_MAV_PAYLOAD_NON_CONST(msg), &packet, 3);
#endif

	msg->msgid = MAVLINK_MSG_ID_ROLL_SIMULATION;
	return mavlink_finalize_message_chan(msg, system_id, component_id, chan, 3, 186);
}

/**
 * @brief Encode a roll_simulation struct into a message
 *
 * @param system_id ID of this system
 * @param component_id ID of this component (e.g. 200 for IMU)
 * @param msg The MAVLink message to compress the data into
 * @param roll_simulation C-struct to read the message contents from
 */
static inline uint16_t mavlink_msg_roll_simulation_encode(uint8_t system_id, uint8_t component_id, mavlink_message_t* msg, const mavlink_roll_simulation_t* roll_simulation)
{
	return mavlink_msg_roll_simulation_pack(system_id, component_id, msg, roll_simulation->roll, roll_simulation->channel);
}

/**
 * @brief Send a roll_simulation message
 * @param chan MAVLink channel to send the message
 *
 * @param roll roll value
 * @param channel roll channel
 */
#ifdef MAVLINK_USE_CONVENIENCE_FUNCTIONS

static inline void mavlink_msg_roll_simulation_send(mavlink_channel_t chan, int16_t roll, uint8_t channel)
{
#if MAVLINK_NEED_BYTE_SWAP || !MAVLINK_ALIGNED_FIELDS
	char buf[3];
	_mav_put_int16_t(buf, 0, roll);
	_mav_put_uint8_t(buf, 2, channel);

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROLL_SIMULATION, buf, 3, 186);
#else
	mavlink_roll_simulation_t packet;
	packet.roll = roll;
	packet.channel = channel;

	_mav_finalize_message_chan_send(chan, MAVLINK_MSG_ID_ROLL_SIMULATION, (const char *)&packet, 3, 186);
#endif
}

#endif

// MESSAGE ROLL_SIMULATION UNPACKING


/**
 * @brief Get field roll from roll_simulation message
 *
 * @return roll value
 */
static inline int16_t mavlink_msg_roll_simulation_get_roll(const mavlink_message_t* msg)
{
	return _MAV_RETURN_int16_t(msg,  0);
}

/**
 * @brief Get field channel from roll_simulation message
 *
 * @return roll channel
 */
static inline uint8_t mavlink_msg_roll_simulation_get_channel(const mavlink_message_t* msg)
{
	return _MAV_RETURN_uint8_t(msg,  2);
}

/**
 * @brief Decode a roll_simulation message into a struct
 *
 * @param msg The message to decode
 * @param roll_simulation C-struct to decode the message contents into
 */
static inline void mavlink_msg_roll_simulation_decode(const mavlink_message_t* msg, mavlink_roll_simulation_t* roll_simulation)
{
#if MAVLINK_NEED_BYTE_SWAP
	roll_simulation->roll = mavlink_msg_roll_simulation_get_roll(msg);
	roll_simulation->channel = mavlink_msg_roll_simulation_get_channel(msg);
#else
	memcpy(roll_simulation, _MAV_PAYLOAD(msg), 3);
#endif
}
