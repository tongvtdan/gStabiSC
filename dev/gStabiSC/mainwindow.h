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
#include "thirdParty/mavlink/v1.0/gMavlinkV1_0.h"
#include "thirdParty/attitude_indicator/attitude_indicator.h"

#include <qwt_plot.h>
#include <qwt_series_data.h>
#include <qwt_plot_curve.h>
#include <qwt_plot_grid.h>
#include <qwt_symbol.h>
#include <qwt_compass.h>
#include <qwt_dial.h>

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

    /**
      for chartPlot
      */
    void chartUpdateData();

private slots:
    void on_upgradeFWButton_clicked();

private Q_SLOTS:
    void onSelectPortChanged(const QString &newPortName);
    void on_SerialPortConnectButton_clicked();
    void onPortAddedRemoved();
    void onReadyReadData();
    void on_clearParamButton_clicked();


    void on_pitchSlider_valueChanged(double value);
    void on_rollSlider_ValueChanged();
    void on_yawknob_ValueChanged();
    void on_yawknob_Released();

    void on_loadfileButtonClicked();
    void on_savefileButtonClicked();
    void init_var(); // init all variables


    void on_ax_checkBox_toggled(bool checked);
    void on_ay_checkBox_toggled(bool checked);
    void on_az_checkBox_toggled(bool checked);
    void on_gx_checkBox_toggled(bool checked);
    void on_gy_checkBox_toggled(bool checked);
    void on_gz_checkBox_toggled(bool checked);
    void on_pitch_checkBox_toggled(bool checked);
    void on_roll_checkBox_toggled(bool checked);
    void on_yaw_checkBox_toggled(bool checked);



signals:
    void messageReceived(QByteArray message);
    void heartBeatPulse(bool heartbeat_status);
    void attitudeChanged(float pitch, float roll, float yaw);
    void paramValueChanged(uint8_t index, float value);
    void sbusValuesChanged();

private:
    Ui::MainWindow *ui;      
    QLabel *m_statusLabel;
    QTimer *watchdogTimer;  
    QTimer *chartTimer;
    AttitudeIndicator *pitch_ai;
    AttitudeIndicator *roll_ai;

    QPolygonF ax_point, ay_point, az_point;
    QPolygonF gx_point, gy_point, gz_point;
    QPolygonF pitch_point, roll_point, yaw_point;

    QwtPlotCurve *ax_curve;
    QwtPlotCurve *ay_curve;
    QwtPlotCurve *az_curve;
    QwtPlotCurve *gx_curve;
    QwtPlotCurve *gy_curve;
    QwtPlotCurve *gz_curve;
    QwtPlotCurve *pitch_curve;
    QwtPlotCurve *roll_curve;
    QwtPlotCurve *yaw_curve;

    double time_count;
    uint32_t interval_value, sampleSize, sampleSizeMax, sampleSizeOverflow ;
    QImage imageOn, imageOff, imageFail;
};

#endif // MAINWINDOW_H
