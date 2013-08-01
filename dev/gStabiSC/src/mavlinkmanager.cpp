#include "mavlinkmanager.h"

#define TARGET_SYSTEM_ID 10

bool hb_pulse = false;

mavlink_attitude_t m_attitude;
mavlink_system_t mavlink_system;

// Define the system type, in this case an airplane
uint8_t system_type = MAV_TYPE_FIXED_WING;
uint8_t autopilot_type = MAV_AUTOPILOT_GENERIC;

uint8_t system_mode = MAV_MODE_PREFLIGHT; ///< Booting up
uint32_t custom_mode = 0;                 ///< Custom mode, can be defined by user/adopter
uint8_t system_state = MAV_STATE_STANDBY; ///< System ready for flight

mavlink_raw_imu_t raw_imu;
mavlink_attitude_t attitude;
mavlink_param_request_read_t request_read;
mavlink_param_value_t paramValue;
mavlink_sbus_chan_values_t sbus_chan_values;
global_struct global_data;
gConfig_t oldParamConfig;



MavlinkManager::MavlinkManager(QObject *parent) :
    QObject(parent)
{
    connect(this, SIGNAL(hb_pulse_changed(bool)), this, SLOT(mavlink_heartbeat_handle(bool)));
}

void MavlinkManager::process_mavlink_msg(QByteArray buff)
{
    mavlink_message_t message;
    mavlink_status_t status;
    unsigned int decodeState;
    uint8_t byte;

    for(int position = 0; position < buff.size(); position++)
    {
        byte = buff[position];
        decodeState = mavlink_parse_char(MAVLINK_COMM_0,byte, &message, &status);

//        if(decodeState) timerRestart();

        switch (message.msgid)
        {
        case MAVLINK_MSG_ID_HEARTBEAT:
            mavlink_heartbeat_t heartbeat;
            heartbeat.mavlink_version = 0;
            mavlink_msg_heartbeat_decode(&message,&heartbeat);
            if(heartbeat.mavlink_version == MAVLINK_VERSION )
                hb_pulse = 1;
            else
                hb_pulse = 0;
            emit hb_pulse_changed(hb_pulse);
            break;
        case MAVLINK_MSG_ID_RAW_IMU:
            raw_imu.xacc = mavlink_msg_raw_imu_get_xacc(&message);
            raw_imu.yacc = mavlink_msg_raw_imu_get_yacc(&message);
            raw_imu.zacc = mavlink_msg_raw_imu_get_zacc(&message);
            raw_imu.xgyro = mavlink_msg_raw_imu_get_xgyro(&message);
            raw_imu.ygyro = mavlink_msg_raw_imu_get_ygyro(&message);
            raw_imu.zgyro = mavlink_msg_raw_imu_get_zgyro(&message);
            break;
        case MAVLINK_MSG_ID_ATTITUDE:
            attitude.roll = mavlink_msg_attitude_get_roll(&message);
            attitude_degree.roll = attitude.roll*180/PI;     // convert to deg
            attitude.pitch = mavlink_msg_attitude_get_pitch(&message);
            attitude_degree.pitch = attitude.pitch*180/PI;
            attitude.yaw = mavlink_msg_attitude_get_yaw(&message);
            attitude_degree.yaw = attitude.yaw*180/PI;
//            emit attitudeChanged(attitude_degree.pitch, attitude_degree.roll, attitude_degree.yaw);
            break;
        case MAVLINK_MSG_ID_PARAM_VALUE:
            paramValue.param_index = mavlink_msg_param_value_get_param_index(&message);  // get param index
            paramValue.param_value = mavlink_msg_param_value_get_param_value(&message);  // get param value
//            emit paramValueChanged(paramValue.param_index, paramValue.param_value);
            break;
        case MAVLINK_MSG_ID_SBUS_CHAN_VALUES:
//            sbus_chan_values.ch1 = mavlink_msg_sbus_chan_values_get_ch1(&message);
//            sbus_chan_values.ch2 = mavlink_msg_sbus_chan_values_get_ch2(&message);
//            sbus_chan_values.ch3 = mavlink_msg_sbus_chan_values_get_ch3(&message);
//            sbus_chan_values.ch4 = mavlink_msg_sbus_chan_values_get_ch4(&message);
//            sbus_chan_values.ch5 = mavlink_msg_sbus_chan_values_get_ch5(&message);
//            sbus_chan_values.ch6 = mavlink_msg_sbus_chan_values_get_ch6(&message);
//            sbus_chan_values.ch7 = mavlink_msg_sbus_chan_values_get_ch7(&message);
//            sbus_chan_values.ch8 = mavlink_msg_sbus_chan_values_get_ch8(&message);
//            sbus_chan_values.ch9 = mavlink_msg_sbus_chan_values_get_ch9(&message);
//            sbus_chan_values.ch10 = mavlink_msg_sbus_chan_values_get_ch10(&message);
//            sbus_chan_values.ch11 = mavlink_msg_sbus_chan_values_get_ch11(&message);
//            sbus_chan_values.ch12 = mavlink_msg_sbus_chan_values_get_ch12(&message);
//            sbus_chan_values.ch13 = mavlink_msg_sbus_chan_values_get_ch13(&message);
//            sbus_chan_values.ch14 = mavlink_msg_sbus_chan_values_get_ch14(&message);
//            sbus_chan_values.ch15 = mavlink_msg_sbus_chan_values_get_ch15(&message);
//            sbus_chan_values.ch16 = mavlink_msg_sbus_chan_values_get_ch16(&message);
//            sbus_chan_values.ch17 = mavlink_msg_sbus_chan_values_get_ch17(&message);
//            sbus_chan_values.ch18 = mavlink_msg_sbus_chan_values_get_ch18(&message);
//            emit updateSbusValues();
            break;
        default:
            break;
        } // end of switch
    }
//     read all params at the first time
//    if(readParams == false){
//        readParamsOnBoard();
//        readParams = true;
//    }

//    if(ui->tabWidget->currentIndex() == 3){
//        chartTimer->start();
//        chartTimerStarted = true;
//    }
//    else{
//        chartTimer->stop();
//        chartTimerStarted = false;
//    }
}


