#ifndef MAVLINKMANAGER_H
#define MAVLINKMANAGER_H

#include <QObject>

#include "thirdParty/mavlink/v1.0/gremsyBGC/mavlink.h"
#include "thirdParty/mavlink/v1.0/globalData.h"
#include "thirdParty/mavlink/v1.0/gMavlinkV1_0.h"

class MavlinkManager : public QObject
{
    Q_OBJECT
public:
    explicit MavlinkManager(QObject *parent = 0);

    mavlink_attitude_t attitude_degree;  // to convert rad to deg
    
signals:
    void hb_pulse_changed(const bool hb);
public slots:
    void process_mavlink_msg(QByteArray buff);
    
};

#endif // MAVLINKMANAGER_H
