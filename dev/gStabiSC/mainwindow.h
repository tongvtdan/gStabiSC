#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <QProcess>
#include <QLabel>
#include <QFileDialog>
#include <QtXml>

//#include <QtSerialPort/QSerialPort>

#include "qextserialport.h"
#include "qextserialenumerator.h"

#include "thirdParty/mavlink/v1.0/gremsyBGC/mavlink.h"
#include "thirdParty/mavlink/v1.0/globalData.h"
#include "thirdParty/attitude_indicator/attitude_indicator.h"

#include <qwt_plot.h>
#include <qwt_series_data.h>
#include <qwt_plot_curve.h>
#include <qwt_plot_grid.h>
#include <qwt_symbol.h>
#include <qwt_compass.h>
#include <qwt_dial.h>

#define BATT_CELL_MIN	3.5
#define BATT_CELL_MAX 	4.2
#define BATT_CELL_ALARM	3.6

#define BATT_OK		0
#define BATT_LOW    1
#define BATT_PC		2

#define BATT_NO_CELL	2
#define BATT_3_CELL		3
#define BATT_4_CELL		4
#define BATT_5_CELL		5
#define BATT_6_CELL		6

class AttitudeIndicator;

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();
    // style for Application
    enum G_MAINWINDOW_STYLE
        {
            G_MAINWINDOW_STYLE_NATIVE,
            G_MAINWINDOW_STYLE_INDOOR,
            G_MAINWINDOW_STYLE_OUTDOOR
        };
    G_MAINWINDOW_STYLE currentStyle;
    QString styleFileName;
    QextSerialPort *serialport;
    QextSerialEnumerator *enumerator;
    QProcess m_process;

    /**
     * @brief Graph Chart functions
     */
    void chartSetting(QwtPlot *plot);
    void attitudeIndicatorSetting();

    void updateXMLfile(QDomDocument document, QString xmlfile);
    void exportXMLfile(QString xmlfilename);
    void importXMLfile(QString importfile);
    void saveXMLfile(QString xmlfile);
    int ListElements(QDomElement root, QString tagname, QString attribute);

    QPalette colorTheme( const QColor & ) const;

//#define PI  3.14;           // define PI value
    mavlink_attitude_t attitude_degree;  // to convert rad to deg
public slots:
    /** @brief Load a specific style */
    void loadAppStyle(G_MAINWINDOW_STYLE style);
    /** @brief Reload the CSS style sheet */
    void reloadAppStylesheet();
    /** @brief Shows an info message as popup or as widget */
    void showInfoMessage(const QString& title, const QString& message);

    void aboutActionTriggered();
    void helpActionTriggered();

    void fillSerialPortInfo();
    void openSerialPort();

    void portSettings();
    /*! Trigger when a device plug or unplug from COM/USB port*/

    void updatePortStatus(bool isConnected);
    /**
     *mavlink protocol process functions
     */
    void handleMessage(QByteArray buff);
    void heartBeatPulseHandle(bool heartbeat_status);
    void updateAttitude(float pitch, float roll, float yaw);
    void updateParamValue(uint8_t index, float value);
    void readParamsOnBoard();
    void writeParamstoBoard();
    void timeOutHandle();
    void timerRestart();
    void setDefaultParams();
    void updateSbusValues();
    void updatePPMValues();
    void updateBatteryStatus(float voltage);

    /**
      for chartPlot
      */
    void chartUpdateData();
    void rcSimulationSend();

private slots:
    void on_upgradeFWButton_clicked();

private Q_SLOTS:
    void SelectPortChanged(const QString &newPortName);
    void on_SerialPortConnectButton_clicked();
    void PortAddedRemoved();
    void onReadyReadData();

//    void on_yawknob_sliderReleased();

    void loadfileButtonClicked();
    void savefileButtonClicked();
    void init_var(); // init all variables

    void on_pitch_checkBox_toggled(bool checked);
    void on_roll_checkBox_toggled(bool checked);
    void on_yaw_checkBox_toggled(bool checked);

    void on_clearParam_clicked();

    void on_calibAcc_Button_clicked();

    void on_calibGyro_Button_clicked();

    void on_rc_source_currentIndexChanged(int index);

    void on_pitchChan_currentIndexChanged(int index);

    void on_handDeviceConnectButton_clicked();

signals:
    void messageReceived(QByteArray message);
    void heartBeatPulse(bool heartbeat_status);
    void attitudeChanged(float pitch, float roll, float yaw);
    void paramValueChanged(uint8_t index, float value);
    void sbusValuesChanged();
    void ppmValuesChanged();
    void baterryValuesChanged(float voltage);

private:
    Ui::MainWindow *ui;      
    QLabel *m_statusLabel;
    QLabel *batteryStatusLabel;
    QTimer *watchdogTimer;  
    QTimer *chartTimer;
    AttitudeIndicator *pitch_ai;
    AttitudeIndicator *roll_ai;

    QPolygonF pitch_point, roll_point, yaw_point;

    QwtPlotCurve *pitch_curve;
    QwtPlotCurve *roll_curve;
    QwtPlotCurve *yaw_curve;

    double time_count;
    uint32_t interval_value, sampleSize, sampleSizeMax, sampleSizeOverflow ;
    QImage imageOn, imageOff, imageFail;
};

#endif // MAINWINDOW_H
