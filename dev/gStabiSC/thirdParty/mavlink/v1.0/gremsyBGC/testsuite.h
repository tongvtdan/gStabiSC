/** @file
 *	@brief MAVLink comm protocol testsuite generated from gremsyBGC.xml
 *	@see http://qgroundcontrol.org/mavlink/
 */
#ifndef GREMSYBGC_TESTSUITE_H
#define GREMSYBGC_TESTSUITE_H

#ifdef __cplusplus
extern "C" {
#endif

#ifndef MAVLINK_TEST_ALL
#define MAVLINK_TEST_ALL
static void mavlink_test_common(uint8_t, uint8_t, mavlink_message_t *last_msg);
static void mavlink_test_gremsyBGC(uint8_t, uint8_t, mavlink_message_t *last_msg);

static void mavlink_test_all(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_common(system_id, component_id, last_msg);
	mavlink_test_gremsyBGC(system_id, component_id, last_msg);
}
#endif

#include "../common/testsuite.h"


static void mavlink_test_ppm_chan_values(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_ppm_chan_values_t packet_in = {
		17.0,
	45.0,
	73.0,
	101.0,
	};
	mavlink_ppm_chan_values_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.tilt = packet_in.tilt;
        	packet1.pan = packet_in.pan;
        	packet1.roll = packet_in.roll;
        	packet1.mode = packet_in.mode;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ppm_chan_values_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_ppm_chan_values_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ppm_chan_values_pack(system_id, component_id, &msg , packet1.tilt , packet1.pan , packet1.roll , packet1.mode );
	mavlink_msg_ppm_chan_values_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ppm_chan_values_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.tilt , packet1.pan , packet1.roll , packet1.mode );
	mavlink_msg_ppm_chan_values_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_ppm_chan_values_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_ppm_chan_values_send(MAVLINK_COMM_1 , packet1.tilt , packet1.pan , packet1.roll , packet1.mode );
	mavlink_msg_ppm_chan_values_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_sbus_chan_values(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_sbus_chan_values_t packet_in = {
		17235,
	17339,
	17443,
	17547,
	17651,
	17755,
	17859,
	17963,
	18067,
	18171,
	18275,
	18379,
	18483,
	18587,
	18691,
	18795,
	18899,
	19003,
	};
	mavlink_sbus_chan_values_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.ch1 = packet_in.ch1;
        	packet1.ch2 = packet_in.ch2;
        	packet1.ch3 = packet_in.ch3;
        	packet1.ch4 = packet_in.ch4;
        	packet1.ch5 = packet_in.ch5;
        	packet1.ch6 = packet_in.ch6;
        	packet1.ch7 = packet_in.ch7;
        	packet1.ch8 = packet_in.ch8;
        	packet1.ch9 = packet_in.ch9;
        	packet1.ch10 = packet_in.ch10;
        	packet1.ch11 = packet_in.ch11;
        	packet1.ch12 = packet_in.ch12;
        	packet1.ch13 = packet_in.ch13;
        	packet1.ch14 = packet_in.ch14;
        	packet1.ch15 = packet_in.ch15;
        	packet1.ch16 = packet_in.ch16;
        	packet1.ch17 = packet_in.ch17;
        	packet1.ch18 = packet_in.ch18;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sbus_chan_values_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_sbus_chan_values_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sbus_chan_values_pack(system_id, component_id, &msg , packet1.ch1 , packet1.ch2 , packet1.ch3 , packet1.ch4 , packet1.ch5 , packet1.ch6 , packet1.ch7 , packet1.ch8 , packet1.ch9 , packet1.ch10 , packet1.ch11 , packet1.ch12 , packet1.ch13 , packet1.ch14 , packet1.ch15 , packet1.ch16 , packet1.ch17 , packet1.ch18 );
	mavlink_msg_sbus_chan_values_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sbus_chan_values_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.ch1 , packet1.ch2 , packet1.ch3 , packet1.ch4 , packet1.ch5 , packet1.ch6 , packet1.ch7 , packet1.ch8 , packet1.ch9 , packet1.ch10 , packet1.ch11 , packet1.ch12 , packet1.ch13 , packet1.ch14 , packet1.ch15 , packet1.ch16 , packet1.ch17 , packet1.ch18 );
	mavlink_msg_sbus_chan_values_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_sbus_chan_values_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_sbus_chan_values_send(MAVLINK_COMM_1 , packet1.ch1 , packet1.ch2 , packet1.ch3 , packet1.ch4 , packet1.ch5 , packet1.ch6 , packet1.ch7 , packet1.ch8 , packet1.ch9 , packet1.ch10 , packet1.ch11 , packet1.ch12 , packet1.ch13 , packet1.ch14 , packet1.ch15 , packet1.ch16 , packet1.ch17 , packet1.ch18 );
	mavlink_msg_sbus_chan_values_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_tilt_simulation(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_tilt_simulation_t packet_in = {
		17235,
	139,
	};
	mavlink_tilt_simulation_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.tilt = packet_in.tilt;
        	packet1.channel = packet_in.channel;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_tilt_simulation_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_tilt_simulation_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_tilt_simulation_pack(system_id, component_id, &msg , packet1.tilt , packet1.channel );
	mavlink_msg_tilt_simulation_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_tilt_simulation_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.tilt , packet1.channel );
	mavlink_msg_tilt_simulation_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_tilt_simulation_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_tilt_simulation_send(MAVLINK_COMM_1 , packet1.tilt , packet1.channel );
	mavlink_msg_tilt_simulation_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_roll_simulation(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_roll_simulation_t packet_in = {
		17235,
	139,
	};
	mavlink_roll_simulation_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.roll = packet_in.roll;
        	packet1.channel = packet_in.channel;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_roll_simulation_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_roll_simulation_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_roll_simulation_pack(system_id, component_id, &msg , packet1.roll , packet1.channel );
	mavlink_msg_roll_simulation_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_roll_simulation_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.roll , packet1.channel );
	mavlink_msg_roll_simulation_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_roll_simulation_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_roll_simulation_send(MAVLINK_COMM_1 , packet1.roll , packet1.channel );
	mavlink_msg_roll_simulation_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_pan_simulation(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_pan_simulation_t packet_in = {
		17235,
	139,
	};
	mavlink_pan_simulation_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.pan = packet_in.pan;
        	packet1.channel = packet_in.channel;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pan_simulation_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_pan_simulation_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pan_simulation_pack(system_id, component_id, &msg , packet1.pan , packet1.channel );
	mavlink_msg_pan_simulation_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pan_simulation_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.pan , packet1.channel );
	mavlink_msg_pan_simulation_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_pan_simulation_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_pan_simulation_send(MAVLINK_COMM_1 , packet1.pan , packet1.channel );
	mavlink_msg_pan_simulation_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_gyro_calib_request(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_gyro_calib_request_t packet_in = {
		5,
	};
	mavlink_gyro_calib_request_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.gyro_calib_mode = packet_in.gyro_calib_mode;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gyro_calib_request_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_gyro_calib_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gyro_calib_request_pack(system_id, component_id, &msg , packet1.gyro_calib_mode );
	mavlink_msg_gyro_calib_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gyro_calib_request_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.gyro_calib_mode );
	mavlink_msg_gyro_calib_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_gyro_calib_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gyro_calib_request_send(MAVLINK_COMM_1 , packet1.gyro_calib_mode );
	mavlink_msg_gyro_calib_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_gyro_calib_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_gyro_calib_status_t packet_in = {
		5,
	};
	mavlink_gyro_calib_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.status = packet_in.status;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gyro_calib_status_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_gyro_calib_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gyro_calib_status_pack(system_id, component_id, &msg , packet1.status );
	mavlink_msg_gyro_calib_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gyro_calib_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.status );
	mavlink_msg_gyro_calib_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_gyro_calib_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_gyro_calib_status_send(MAVLINK_COMM_1 , packet1.status );
	mavlink_msg_gyro_calib_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_acc_calib_request(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_acc_calib_request_t packet_in = {
		5,
	};
	mavlink_acc_calib_request_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.acc_calib_mode = packet_in.acc_calib_mode;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_acc_calib_request_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_acc_calib_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_acc_calib_request_pack(system_id, component_id, &msg , packet1.acc_calib_mode );
	mavlink_msg_acc_calib_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_acc_calib_request_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.acc_calib_mode );
	mavlink_msg_acc_calib_request_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_acc_calib_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_acc_calib_request_send(MAVLINK_COMM_1 , packet1.acc_calib_mode );
	mavlink_msg_acc_calib_request_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_acc_calib_status(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_message_t msg;
        uint8_t buffer[MAVLINK_MAX_PACKET_LEN];
        uint16_t i;
	mavlink_acc_calib_status_t packet_in = {
		5,
	};
	mavlink_acc_calib_status_t packet1, packet2;
        memset(&packet1, 0, sizeof(packet1));
        	packet1.acc_calib_status = packet_in.acc_calib_status;
        
        

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_acc_calib_status_encode(system_id, component_id, &msg, &packet1);
	mavlink_msg_acc_calib_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_acc_calib_status_pack(system_id, component_id, &msg , packet1.acc_calib_status );
	mavlink_msg_acc_calib_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_acc_calib_status_pack_chan(system_id, component_id, MAVLINK_COMM_0, &msg , packet1.acc_calib_status );
	mavlink_msg_acc_calib_status_decode(&msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);

        memset(&packet2, 0, sizeof(packet2));
        mavlink_msg_to_send_buffer(buffer, &msg);
        for (i=0; i<mavlink_msg_get_send_buffer_length(&msg); i++) {
        	comm_send_ch(MAVLINK_COMM_0, buffer[i]);
        }
	mavlink_msg_acc_calib_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
        
        memset(&packet2, 0, sizeof(packet2));
	mavlink_msg_acc_calib_status_send(MAVLINK_COMM_1 , packet1.acc_calib_status );
	mavlink_msg_acc_calib_status_decode(last_msg, &packet2);
        MAVLINK_ASSERT(memcmp(&packet1, &packet2, sizeof(packet1)) == 0);
}

static void mavlink_test_gremsyBGC(uint8_t system_id, uint8_t component_id, mavlink_message_t *last_msg)
{
	mavlink_test_ppm_chan_values(system_id, component_id, last_msg);
	mavlink_test_sbus_chan_values(system_id, component_id, last_msg);
	mavlink_test_tilt_simulation(system_id, component_id, last_msg);
	mavlink_test_roll_simulation(system_id, component_id, last_msg);
	mavlink_test_pan_simulation(system_id, component_id, last_msg);
	mavlink_test_gyro_calib_request(system_id, component_id, last_msg);
	mavlink_test_gyro_calib_status(system_id, component_id, last_msg);
	mavlink_test_acc_calib_request(system_id, component_id, last_msg);
	mavlink_test_acc_calib_status(system_id, component_id, last_msg);
}

#ifdef __cplusplus
}
#endif // __cplusplus
#endif // GREMSYBGC_TESTSUITE_H
