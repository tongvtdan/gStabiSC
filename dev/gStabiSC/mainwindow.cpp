#include <QMessageBox>

#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "qextserialport.h"
#include "qextserialenumerator.h"
#include <QDebug>
#include <QTime>
#include <QTimer>

//#define SYSTEM_ID 20
#define TARGET_SYSTEM_ID 10

bool pulse = false;
bool readParams = false;
bool chartTimerStarted = false;

QTime logtime;
QTimer sendTimer;
QString loadfilename, savefilename, filedir;
QString oldprofilename;

mavlink_attitude_t att;
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


MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    m_statusLabel(new QLabel)


{
    ui->setupUi(this);

//    init all required variables
    init_var();

    loadAppStyle(currentStyle);


    connect(ui->actionAbout,SIGNAL(triggered()),  this, SLOT(aboutActionTriggered()));
    connect(ui->actionHelp, SIGNAL(triggered()),this, SLOT(helpActionTriggered()));


    fillSerialPortInfo();

    portSettings();

    connect(ui->portListBox, SIGNAL(currentIndexChanged(QString)),SLOT(onSelectPortChanged(QString)));

    connect(enumerator, SIGNAL(deviceDiscovered(QextPortInfo)), SLOT(onPortAddedRemoved()));
    connect(enumerator, SIGNAL(deviceRemoved(QextPortInfo)), SLOT(onPortAddedRemoved()));

    connect(serialport, SIGNAL(readyRead()),this, SLOT(onReadyReadData()));
    connect(this, SIGNAL(messageReceived(QByteArray)),this, SLOT(handleMessage(QByteArray)));
    connect(this,SIGNAL(heartBeatPulse(bool)),this,SLOT(heartBeatPulseHandle(bool)));

    ui->tabWidget->setCurrentIndex(0);

    m_statusLabel->setText(QString("%1: 57600, N, 8, 1").arg(serialport->portName()));
    m_statusLabel->setMinimumWidth(200);
    ui->statusBar->addWidget(m_statusLabel);

    //open profile file
    connect(ui->loadfileButton, SIGNAL(clicked()), SLOT(on_loadfileButtonClicked()));
    //save profile file
    connect(ui->savefileButton, SIGNAL(clicked()), SLOT(on_savefileButtonClicked()));

    //set default all params values
    connect(ui->setDefaultParams, SIGNAL(clicked()), SLOT(setDefaultParams()));

    //clear all params
//    connect(ui->clearParam, SIGNAL(clicked()), SLOT(on_clearParamButton_clicked()));

    //write param to board
    connect(ui->writeParam, SIGNAL(clicked()), SLOT(writeParamstoBoard()));

    //read param on board
    connect(ui->readParam, SIGNAL(clicked()), SLOT(readParamsOnBoard()));
    //param value update
    connect(this, SIGNAL(paramValueChanged(uint8_t, float)), this, SLOT(updateParamValue(uint8_t, float)));

    // attitude display
    connect(this, SIGNAL(attitudeChanged(float, float, float)),this, SLOT(updateAttitude(float, float, float)));

    /* connect line chart check box */
    connect(ui->ax_checkBox, SIGNAL(toggled(bool)), SLOT(on_ax_CheckBox_toggled(bool)));
    connect(ui->ay_checkBox, SIGNAL(toggled(bool)), SLOT(on_ay_CheckBox_toggled(bool)));
    connect(ui->az_checkBox, SIGNAL(toggled(bool)), SLOT(on_az_CheckBox_toggled(bool)));
    connect(ui->gx_checkBox, SIGNAL(toggled(bool)), SLOT(on_gx_CheckBox_toggled(bool)));
    connect(ui->gy_checkBox, SIGNAL(toggled(bool)), SLOT(on_gy_CheckBox_toggled(bool)));
    connect(ui->gz_checkBox, SIGNAL(toggled(bool)), SLOT(on_gz_CheckBox_toggled(bool)));
    connect(ui->pitch_checkBox, SIGNAL(toggled(bool)), SLOT(on_pitch_CheckBox_toggled(bool)));
    connect(ui->roll_checkBox, SIGNAL(toggled(bool)), SLOT(on_roll_CheckBox_toggled(bool)));
    connect(ui->yaw_checkBox, SIGNAL(toggled(bool)), SLOT(on_yaw_CheckBox_toggled(bool)));

    watchdogTimer = new QTimer(this);
    chartTimer = new QTimer(this);

    connect(watchdogTimer,SIGNAL(timeout()), this, SLOT(timeOutHandle()));
    connect(chartTimer, SIGNAL(timeout()), SLOT(chartUpdateData()));

    connect(this, SIGNAL(sbusValuesChanged()), this, SLOT(updateSbusValues()));

    connect(ui->pitchSlider, SIGNAL(valueChanged(double)), SLOT(on_pitchSlider_ValueChanged()));
    connect(ui->rollSlider, SIGNAL(valueChanged(double)), SLOT(on_rollSlider_ValueChanged()));
    connect(ui->yawknob, SIGNAL(valueChanged(double)), SLOT(on_yawknob_ValueChanged()));
    connect(ui->yawknob, SIGNAL(sliderReleased()),SLOT(on_yawknob_Released()));

    chartSetting();

    attitudeIndicatorSetting();
}

MainWindow::~MainWindow()
{
    delete serialport;
    delete ui;
}

void MainWindow::chartSetting()
{
    QwtPlotGrid *grid = new QwtPlotGrid();
    ui->chartPlot->setAutoReplot(true);

    grid->setMinorPen(QPen(Qt::gray, 0, Qt::DotLine));
    grid->setMajorPen(QPen(Qt::gray, 0, Qt::DotLine));
    grid->enableX(true);
    grid->enableY(true);
    grid->attach(ui->chartPlot);

    ax_curve = new QwtPlotCurve();
    ax_curve->setTitle("AccX");
    ax_curve->setPen(Qt::red, 2, Qt::SolidLine);
    ax_curve->setRenderHint(QwtPlotItem::RenderAntialiased, true);
    ax_curve->attach(ui->chartPlot);
    ax_curve->hide();

    ay_curve = new QwtPlotCurve();
    ay_curve->setTitle("AccY");
    ay_curve->setPen(Qt::green, 2, Qt::SolidLine);
    ay_curve->setRenderHint(QwtPlotItem::RenderAntialiased, true);
    ay_curve->attach(ui->chartPlot);
    ay_curve->hide();

    az_curve = new QwtPlotCurve();
    az_curve->setTitle("AccZ");
    az_curve->setPen(Qt::blue, 2, Qt::SolidLine);
    az_curve->setRenderHint(QwtPlotItem::RenderAntialiased, true);
    az_curve->attach(ui->chartPlot);
    az_curve->hide();

    gx_curve = new QwtPlotCurve();
    gx_curve->setTitle("GyroX");
    gx_curve->setPen(Qt::magenta, 2, Qt::SolidLine);
    gx_curve->setRenderHint(QwtPlotItem::RenderAntialiased, true);
    gx_curve->attach(ui->chartPlot);
    gx_curve->hide();

    gy_curve = new QwtPlotCurve();
    gy_curve->setTitle("GyroY");
    gy_curve->setPen(Qt::darkBlue, 2, Qt::SolidLine);
    gy_curve->setRenderHint(QwtPlotItem::RenderAntialiased, true);
    gy_curve->attach(ui->chartPlot);
    gy_curve->hide();

    gz_curve = new QwtPlotCurve();
    gz_curve->setTitle("GyroZ");
    gz_curve->setPen(Qt::darkCyan, 2, Qt::SolidLine);
    gz_curve->setRenderHint(QwtPlotItem::RenderAntialiased, true);
    gz_curve->attach(ui->chartPlot);
    gz_curve->hide();

    pitch_curve = new QwtPlotCurve();
    pitch_curve->setTitle("Pitch in degree");
    pitch_curve->setPen(Qt::darkRed, 2, Qt::SolidLine);
    pitch_curve->setRenderHint(QwtPlotItem::RenderAntialiased, true);
    pitch_curve->attach(ui->chartPlot);
    ui->pitch_checkBox->setChecked(true);
    pitch_curve->show();

    roll_curve = new QwtPlotCurve();
    roll_curve->setTitle("Roll in degree");
    roll_curve->setPen(Qt::darkGreen, 2, Qt::SolidLine);
    roll_curve->setRenderHint(QwtPlotItem::RenderAntialiased, true);
    roll_curve->attach(ui->chartPlot);
    ui->roll_checkBox->setChecked(true);
    roll_curve->show();

    yaw_curve = new QwtPlotCurve();
    yaw_curve->setTitle("Yaw in degree");
    yaw_curve->setPen(Qt::darkMagenta, 2, Qt::SolidLine);
    yaw_curve->setRenderHint(QwtPlotItem::RenderAntialiased, true);
    yaw_curve->attach(ui->chartPlot);
    ui->yaw_checkBox->setChecked(true);
    yaw_curve->show();
}

void MainWindow::attitudeIndicatorSetting()
{
    /* khoi tao attitude cockpit cho truc roll */
//    setPalette( colorTheme( QColor( Qt::darkGray ).dark( 150 ) ) );
    QGridLayout *layout = new QGridLayout( this );
    layout->setSpacing( 5 );
    layout->setMargin( 0 );

    QwtDial *dial = NULL;

    roll_ai = new AttitudeIndicator( this );
    roll_ai->scaleDraw()->setPenWidth( 2 );
    roll_ai->scaleDraw()->setAngleRange(0, 360);
    roll_ai->setPalette(colorTheme( QColor( Qt::gray).dark(250)));
    roll_ai->setStyleSheet("color: rgb(6,228,249) ;");
    roll_ai->setGeometry(20, 306, 120, 120);
    dial = roll_ai;

    if( dial )
    {
        dial->setReadOnly( true );
        dial->setLineWidth(4);
        dial->setFrameShadow( QwtDial::Plain );
    }

    layout->addWidget( dial, 0, 0 );

    /*
    khoi tao attitude cockpit cho truc pitch
    */

    QwtCompassScaleDraw *pitchscaleDraw = new QwtCompassScaleDraw();
    pitchscaleDraw->enableComponent( QwtAbstractScaleDraw::Ticks, true );
    pitchscaleDraw->enableComponent( QwtAbstractScaleDraw::Labels, false );
    pitchscaleDraw->enableComponent( QwtAbstractScaleDraw::Backbone, true );

    pitchscaleDraw->setTickLength( QwtScaleDiv::MinorTick, 0 );
    pitchscaleDraw->setTickLength( QwtScaleDiv::MediumTick, 3 );
    pitchscaleDraw->setTickLength( QwtScaleDiv::MajorTick, 10 );
    pitchscaleDraw->setPenWidth(2);

    ui->pitchCompass->setScaleDraw( pitchscaleDraw );

    ui->pitchCompass->setPalette(colorTheme( QColor( Qt::green).dark(250)));
    ui->pitchCompass->setStyleSheet("color: rgb(6,228,249) ;");
    ui->pitchCompass->setNeedle( new QwtDialSimpleNeedle( QwtDialSimpleNeedle::Arrow, true,
        Qt::red, Qt::blue ) );
    ui->pitchCompass->setValue( 90 );

    /*
    khoi tao attitude cockpit cho truc yaw
    */
//    QMap<double, QString> map;
//    for ( double d = 0.0; d < 360.0; d += 60.0 )
//    {
//        QString label;
//        label.sprintf( "%.0f", d );
//        map.insert( d, label );
//    }

    QwtCompassScaleDraw *yawscaleDraw = new QwtCompassScaleDraw();
    yawscaleDraw->enableComponent( QwtAbstractScaleDraw::Ticks, true );
    yawscaleDraw->enableComponent( QwtAbstractScaleDraw::Labels, true );
    yawscaleDraw->enableComponent( QwtAbstractScaleDraw::Backbone, true );

    yawscaleDraw->setTickLength( QwtScaleDiv::MinorTick, 0 );
    yawscaleDraw->setTickLength( QwtScaleDiv::MediumTick, 3 );
    yawscaleDraw->setTickLength( QwtScaleDiv::MajorTick, 10 );
    yawscaleDraw->setPenWidth(2);

    ui->yawCompass->setScaleDraw( yawscaleDraw );
    ui->yawCompass->setPalette(colorTheme( QColor( Qt::green).dark(250)));
    ui->yawCompass->setStyleSheet("color: rgb(6,228,249) ;");
    ui->yawCompass->setNeedle( new QwtCompassMagnetNeedle( QwtCompassMagnetNeedle::TriangleStyle,
                                                        Qt::blue, Qt::red ) );
    ui->yawCompass->setValue( 0 );
}

void MainWindow::updateXMLfile(QDomDocument document, QString xmlfile)
{
    QDomElement groups = document.createElement("Groups");
    document.appendChild(groups);

        /* motors params */
        QDomElement motors = document.createElement("Motors");
        motors.setAttribute("Freq", ui->motor_freq->currentIndex());
        groups.appendChild(motors);
            QDomElement pitchmotor = document.createElement("PitchMotor");
{
                /*pitch motor elements*/
                pitchmotor.setAttribute("Power", ui->pitch_Power->value());
                pitchmotor.setAttribute("Poles", ui->pitch_Pole->value());
                pitchmotor.setAttribute("MaxTravel", ui->pitch_maxTravel->value());
                pitchmotor.setAttribute("MinTravel", ui->pitch_minTravel->value());
                pitchmotor.setAttribute("Direction", ui->pitch_Dir->currentIndex());
            motors.appendChild(pitchmotor);
                /*end of pitch motor elements*/
}
            QDomElement rollmotor = document.createElement("RollMotor");
{
                /*roll motor elements*/
                rollmotor.setAttribute("Power", ui->roll_Power->value());
                rollmotor.setAttribute("Poles", ui->roll_Pole->value());
                rollmotor.setAttribute("MaxTravel", ui->roll_maxTravel->value());
                rollmotor.setAttribute("MinTravel", ui->roll_minTravel->value());
                rollmotor.setAttribute("Direction", ui->roll_Dir->currentIndex());
            motors.appendChild(rollmotor);
                /*end of roll motor elements*/
}
            QDomElement yawmotor = document.createElement("YawMotor");
{
                /*roll motor elements*/
                yawmotor.setAttribute("Power", ui->yaw_Power->value());
                yawmotor.setAttribute("Poles", ui->yaw_Pole->value());
                yawmotor.setAttribute("MaxTravel", ui->yaw_maxTravel->value());
                yawmotor.setAttribute("MinTravel", ui->yaw_minTravel->value());
                yawmotor.setAttribute("Direction", ui->yaw_Dir->currentIndex());
            motors.appendChild(yawmotor);
                /*end of roll motor elements*/
}
        /* end of motors params */

        /* PID params */
        QDomElement pids = document.createElement("PIDs");
        groups.appendChild(pids);
            QDomElement pitchpid = document.createElement("PitchPID");
            {
                pitchpid.setAttribute("P", ui->pitch_P->value());
                pitchpid.setAttribute("I", ui->pitch_I->value());
                pitchpid.setAttribute("D", ui->pitch_D->value());
                pids.appendChild(pitchpid);
            }

            QDomElement rollpid = document.createElement("RollPID");
            {
                rollpid.setAttribute("P", ui->roll_P->value());
                rollpid.setAttribute("I", ui->roll_I->value());
                rollpid.setAttribute("D", ui->roll_D->value());
                pids.appendChild(rollpid);
            }

            QDomElement yawpid = document.createElement("YawPID");
            {
                yawpid.setAttribute("P", ui->yaw_P->value());
                yawpid.setAttribute("I", ui->yaw_I->value());
                yawpid.setAttribute("D", ui->yaw_D->value());
                pids.appendChild(yawpid);
            }
        /* end of pids params */

        /* follow mode params */
        QDomElement follows = document.createElement("Follows");
        groups.appendChild(follows);
            QDomElement pitchfollow = document.createElement("PitchFollow");
            {
                pitchfollow.setAttribute("Follow", ui->follow_pitch->value());
                pitchfollow.setAttribute("Filter", ui->pitch_filter->value());
                follows.appendChild(pitchfollow);
            }

            QDomElement rollfollow = document.createElement("RollFollow");
            {
                rollfollow.setAttribute("Follow", ui->follow_roll->value());
                rollfollow.setAttribute("Filter", ui->roll_filter->value());
                follows.appendChild(rollfollow);
            }

            QDomElement yawfollow = document.createElement("YawFollow");
            {
                yawfollow.setAttribute("Follow", ui->follow_yaw->value());
                yawfollow.setAttribute("Filter", ui->yaw_filter->value());
                follows.appendChild(yawfollow);
            }

            /* end of follow mode params */

        /* IMU params */
        QDomElement imu = document.createElement("IMU");
        groups.appendChild(imu);
            QDomElement gyro = document.createElement("Gyro");
            {
                gyro.setAttribute("Xoffset", ui->gyroX_offset->value());
                gyro.setAttribute("Yoffset", ui->gyroY_offset->value());
                gyro.setAttribute("Zoffset", ui->gyroZ_offset->value());
                gyro.setAttribute("GyroTrust", ui->gyroTrust->value());
                gyro.setAttribute("GyroLPF", ui->gyro_LPF->value());
                gyro.setAttribute("SkipCalibGyro", ui->calibGyro->checkState());
                imu.appendChild(gyro);
            }

            QDomElement acc = document.createElement("Acc");
            {
                acc.setAttribute("Xoffset", ui->accX_offset->value());
                acc.setAttribute("Yoffset", ui->accY_offset->value());
                acc.setAttribute("Zoffset", ui->accZ_offset->value());
                imu.appendChild(acc);
            }

            QDomElement gps = document.createElement("GPS");
            {
                gps.setAttribute("UseGPS", ui->useGPS->checkState());
                imu.appendChild(gps);
            }
        /* end of IMU params */

        /* rc params */
        QDomElement rc = document.createElement("RC");
        rc.setAttribute("Source", ui->rc_source->currentIndex());
        rc.setAttribute("StickChan", ui->modeChan->currentIndex());
        groups.appendChild(rc);
            QDomElement pitchchan = document.createElement("PitchChan");
            {
                pitchchan.setAttribute("Chan", ui->pitchChan->currentIndex());
                pitchchan.setAttribute("LPF", ui->rcLPF_pitch->value());
                pitchchan.setAttribute("Trim", ui->trim_pitch->value());
                pitchchan.setAttribute("Mode", ui->mode_pitch->currentIndex());
                rc.appendChild(pitchchan);
            }

            QDomElement rollchan = document.createElement("RollChan");
            {
                rollchan.setAttribute("Chan", ui->rollChan->currentIndex());
                rollchan.setAttribute("LPF", ui->rcLPF_roll->value());
                rollchan.setAttribute("Trim", ui->trim_roll->value());
                rollchan.setAttribute("Mode", ui->mode_roll->currentIndex());
                rc.appendChild(rollchan);
            }

            QDomElement yawchan = document.createElement("YawChan");
            {
                yawchan.setAttribute("Chan", ui->yawChan->currentIndex());
                yawchan.setAttribute("LPF", ui->rcLPF_yaw->value());
                yawchan.setAttribute("Trim", ui->trim_yaw->value());
                yawchan.setAttribute("Mode", ui->mode_yaw->currentIndex());
                rc.appendChild(yawchan);
            }
        /* end of rc params */

        QFile file("profiles/" + xmlfile);

        if(!file.open(QIODevice::WriteOnly | QIODevice::Text))
        {
            qDebug() << "Fail to open file for writting";
        }
        else
        {
            QTextStream stream(&file);
            stream << document.toString();
            file.close();
            qDebug() << "Finished";
        }

}

void MainWindow::exportXMLfile(QString xmlfilename)
{
    QDomDocument exportProfile;

    updateXMLfile(exportProfile, xmlfilename);
}

void MainWindow::importXMLfile(QString importfile)
{
    QDomDocument importProfile;

    //Load the file
    QFile file(importfile);
    if(!file.open(QIODevice::ReadOnly | QIODevice::Text))
    {
        qDebug() << "Failed to open the file";
    }
    else
    {
        if(!importProfile.setContent(&file))
        {
            qDebug() << "Failed to load document";
        }
        file.close();
    }

    //Get the root element
    QDomElement root = importProfile.firstChildElement();

    /* Get the elements */
    //motor frequency
    ui->motor_freq->setCurrentIndex(ListElements(root, "Motors", "Freq"));

    // pitch motor
    ui->pitch_Power->setValue(ListElements(root, "PitchMotor", "Power"));
    ui->pitch_Pole->setValue(ListElements(root, "PitchMotor", "Poles"));
    ui->pitch_maxTravel->setValue(ListElements(root, "PitchMotor", "MaxTravel"));
    ui->pitch_minTravel->setValue(ListElements(root, "PitchMotor", "MinTravel"));
    ui->pitch_Dir->setCurrentIndex(ListElements(root, "PitchMotor", "Direction"));

    // roll motor
    ui->roll_Power->setValue(ListElements(root, "RollMotor", "Power"));
    ui->roll_Pole->setValue(ListElements(root, "RollMotor", "Poles"));
    ui->roll_maxTravel->setValue(ListElements(root, "RollMotor", "MaxTravel"));
    ui->roll_minTravel->setValue(ListElements(root, "RollMotor", "MinTravel"));
    ui->roll_Dir->setCurrentIndex(ListElements(root, "RollMotor", "Direction"));

    // yaw motor
    ui->yaw_Power->setValue(ListElements(root, "YawMotor", "Power"));
    ui->yaw_Pole->setValue(ListElements(root, "YawMotor", "Poles"));
    ui->yaw_maxTravel->setValue(ListElements(root, "YawMotor", "MaxTravel"));
    ui->yaw_minTravel->setValue(ListElements(root, "YawMotor", "MinTravel"));
    ui->yaw_Dir->setCurrentIndex(ListElements(root, "YawMotor", "Direction"));

    // pitch PID
    ui->pitch_P->setValue(ListElements(root, "PitchPID", "P"));
    ui->pitch_I->setValue(ListElements(root, "PitchPID", "I"));
    ui->pitch_D->setValue(ListElements(root, "PitchPID", "D"));

    // roll PID
    ui->roll_P->setValue(ListElements(root, "RollPID", "P"));
    ui->roll_I->setValue(ListElements(root, "RollPID", "I"));
    ui->roll_D->setValue(ListElements(root, "RollPID", "D"));

    // yaw PID
    ui->yaw_P->setValue(ListElements(root, "YawPID", "P"));
    ui->yaw_I->setValue(ListElements(root, "YawPID", "I"));
    ui->yaw_D->setValue(ListElements(root, "YawPID", "D"));

    // pitch follow
    ui->follow_pitch->setValue(ListElements(root, "PitchFollow", "Follow"));
    ui->pitch_filter->setValue(ListElements(root, "PitchFollow", "Filter"));

    // roll follow
    ui->follow_roll->setValue(ListElements(root, "RollFollow", "Follow"));
    ui->roll_filter->setValue(ListElements(root, "RollFollow", "Filter"));

    // yaw follow
    ui->follow_yaw->setValue(ListElements(root, "YawFollow", "Follow"));
    ui->yaw_filter->setValue(ListElements(root, "YawFollow", "Filter"));

    // Gyro
    ui->gyroX_offset->setValue(ListElements(root, "Gyro", "Xoffset"));
    ui->gyroY_offset->setValue(ListElements(root, "Gyro", "Yoffset"));
    ui->gyroZ_offset->setValue(ListElements(root, "Gyro", "Zoffset"));
    ui->gyroTrust->setValue(ListElements(root, "Gyro", "GyroTrust"));
    ui->gyro_LPF->setValue(ListElements(root, "Gyro", "GyroLPF"));
    ui->calibGyro->setChecked(ListElements(root, "Gyro", "SkipCalibGyro"));

    // acc
    ui->accX_offset->setValue(ListElements(root, "Acc", "Xoffset"));
    ui->accY_offset->setValue(ListElements(root, "Acc", "Yoffset"));
    ui->accZ_offset->setValue(ListElements(root, "Acc", "Zoffset"));

    // gps
    ui->useGPS->setChecked(ListElements(root, "GPS", "UseGPS"));

    // rc pitch
    ui->pitchChan->setCurrentIndex(ListElements(root, "PitchChan", "Chan"));
    ui->rcLPF_pitch->setValue(ListElements(root, "PitchChan", "LPF"));
    ui->trim_pitch->setValue(ListElements(root, "PitchChan", "Trim"));
    ui->mode_pitch->setCurrentIndex(ListElements(root, "PitchChan", "Mode"));

    // rc roll
    ui->rollChan->setCurrentIndex(ListElements(root, "RollChan", "Chan"));
    ui->rcLPF_roll->setValue(ListElements(root, "RollChan", "LPF"));
    ui->trim_roll->setValue(ListElements(root, "RollChan", "Trim"));
    ui->mode_roll->setCurrentIndex(ListElements(root, "RollChan", "Mode"));

    // rc yaw
    ui->yawChan->setCurrentIndex(ListElements(root, "YawChan", "Chan"));
    ui->rcLPF_yaw->setValue(ListElements(root, "YawChan", "LPF"));
    ui->trim_yaw->setValue(ListElements(root, "YawChan", "Trim"));
    ui->mode_yaw->setCurrentIndex(ListElements(root, "YawChan", "Mode"));

    // rc type
    ui->rc_source->setCurrentIndex(ListElements(root, "RC", "Source"));

    // Stick channel
    ui->modeChan->setCurrentIndex(ListElements(root, "RC", "StickChan"));

}

void MainWindow::saveXMLfile(QString xmlfile)
{
    QDomDocument profile;

    updateXMLfile(profile, xmlfile);
}

int MainWindow::ListElements(QDomElement root, QString tagname, QString attribute)
{
    QDomNodeList items = root.elementsByTagName(tagname);

    qDebug() << "Total items = " << items.count();

    for(int i=0; i< items.count(); i++)
    {
        QDomNode itemnode = items.at(i);

        //convert to element
        if(itemnode.isElement())
        {
            QDomElement itemele = itemnode.toElement();          
            qDebug() << itemele.attribute(attribute).toInt();
            return itemele.attribute(attribute).toInt();
        }
        else return -1;
    }
}

void MainWindow::loadAppStyle(MainWindow::G_MAINWINDOW_STYLE style)
{
    switch (style) {
    case G_MAINWINDOW_STYLE_NATIVE:
        // Native mode means setting no style
        // so if we were already in native mode
        // take no action
        // Only if a style was set, remove it.
        if (style != currentStyle) {
            qApp->setStyleSheet("");
            showInfoMessage(tr("Please restart Application"), tr("Please restart Application to switch to fully native look and feel. Currently you have loaded Qt's plastique style."));
        }
        break;
    case G_MAINWINDOW_STYLE_INDOOR:
        qApp->setStyle("fusion");
        styleFileName = ":general/files/styles/style-indoor.css";
        reloadAppStylesheet();
        break;
    case G_MAINWINDOW_STYLE_OUTDOOR:
        qApp->setStyle("fusion");
        styleFileName = ":general/files/styles/style-outdoor.css";
        reloadAppStylesheet();
        break;
    default:
        break;
        currentStyle = style;
    }
}

void MainWindow::reloadAppStylesheet()
{
    // Load style sheet
    QFile* styleSheet = new QFile(styleFileName);
    if (!styleSheet->exists())
    {
        styleSheet = new QFile(":general/files/styles/style-indoor.css");
    }
    if (styleSheet->open(QIODevice::ReadOnly | QIODevice::Text))
    {
        QString style = QString(styleSheet->readAll());
        style.replace("ICONDIR", QCoreApplication::applicationDirPath()+ "general/files/styles/");
        qApp->setStyleSheet(style);
    }
    else
    {
        QMessageBox msgBox;
        msgBox.setIcon(QMessageBox::Information);
        msgBox.setText(tr("Application did lot load a new style"));
        msgBox.setInformativeText(tr("Stylesheet file %1 was not readable").arg(styleFileName));
        msgBox.setStandardButtons(QMessageBox::Ok);
        msgBox.setDefaultButton(QMessageBox::Ok);
        msgBox.exec();
    }
    delete styleSheet;
}

void MainWindow::showInfoMessage(const QString &title, const QString &message)
{
    QMessageBox msgBox(this);
    msgBox.setIcon(QMessageBox::Information);
    msgBox.setText(title);
    msgBox.setInformativeText(message);
    msgBox.setStandardButtons(QMessageBox::Ok);
    msgBox.setDefaultButton(QMessageBox::Ok);
    msgBox.exec();
}

void MainWindow::aboutActionTriggered()
{
    QMessageBox::about(this,"About","This application is used for setup gimbal controller from Gremsy Co., Ltd");
}

void MainWindow::helpActionTriggered()
{
    QMessageBox::information(this,"Help","You need help or want to give us feedback,\n please feel free to contact to us by email: \n\
                                  - Hardware: huy.pham@gremsy.com \n\
                                  - Software: dan.tong@gremsy.com\n\
                                  - Others:  contact@gremsy.com \n\
                             Thank you!");
}

void MainWindow::fillSerialPortInfo()
{
       // Get the ports available on this system
      QList<QextPortInfo> ports = QextSerialEnumerator::getPorts();
      // Add the ports in reverse order, because we prepend them to the list
      for(int i = ports.size() - 1; i >= 0; i--){
          QextPortInfo portInfo = ports.at(i);
          ui->portListBox->addItem(portInfo.portName);
      }
      ui->portListBox->setCurrentIndex(0);
}

void MainWindow::openSerialPort()
{
    if(serialport->isOpen())
    {
        serialport->close();
        watchdogTimer->stop();
        chartTimer->stop();
        ui->BoardConnectionStatusLabel->hide();
    }
    else
    {
        serialport->open(QIODevice::ReadWrite);
        watchdogTimer->start();
        ui->BoardConnectionStatusLabel->show();

        serialport->setRts(1); // 0V output on boot0
        serialport->setDtr(1); // 0v output on reset
        serialport->setDtr(0); // 3V3 output on reset
    }
    updatePortStatus(serialport->isOpen());
}

void MainWindow::portSettings()
{
    PortSettings settings = {BAUD57600, DATA_8, PAR_NONE, STOP_1, FLOW_OFF, 100};
    QString t_portName = ui->portListBox->currentText();
    serialport = new QextSerialPort(t_portName, settings, QextSerialPort::EventDriven);
    enumerator = new QextSerialEnumerator(this);
    enumerator->setUpNotifications();
}

void MainWindow::onPortAddedRemoved()
{
    updatePortStatus(false);
    watchdogTimer->stop();
    chartTimer->stop();
    ui->BoardConnectionStatusLabel->hide();

    ui->portListBox->blockSignals(true);
    ui->portListBox->clear();
    fillSerialPortInfo();
    ui->portListBox->setCurrentIndex(0);
    ui->portListBox->blockSignals(false);
    // update portname
    serialport->setPortName(ui->portListBox->currentText());

}

void MainWindow::onReadyReadData()
{
    QByteArray messageData;
    if (serialport->bytesAvailable()) {
        messageData = serialport->readAll();
        serialport->flush();
        emit messageReceived(messageData);
    }
}

void MainWindow::updatePortStatus(bool isConnected){
    if(isConnected)
    {
        ui->SerialPortConnectButton->setText("Disconnect");
        qDebug(("Port Opened"));
        m_statusLabel->setText(QString("%1: 57600, N, 8, 1 - Connected").arg(serialport->portName()));
    }
    else
    {
        ui->SerialPortConnectButton->setText("Connect");
        qDebug("Port Closed");
        ui->hearbeatPulseLabel->setPixmap(QPixmap::fromImage(imageOff));
        m_statusLabel->setText(QString("%1: 57600, N, 8, 1 - Disconnected").arg(serialport->portName()));
    }
}

void MainWindow::handleMessage(QByteArray buff)
{
    mavlink_message_t message;
    mavlink_status_t status;
    unsigned int decodeState;
    uint8_t byte;

    for(int position = 0; position < buff.size(); position++)
    {
        byte = buff[position];
        decodeState = mavlink_parse_char(MAVLINK_COMM_0,byte, &message, &status);

        if(decodeState) timerRestart();

        switch (message.msgid)
        {
        case MAVLINK_MSG_ID_HEARTBEAT:
            mavlink_heartbeat_t heartbeat;
            heartbeat.mavlink_version = 0;
            mavlink_msg_heartbeat_decode(&message,&heartbeat);
            if(heartbeat.mavlink_version == MAVLINK_VERSION )
                pulse = 1;
            else
                pulse = 0;
            emit heartBeatPulse(pulse);
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
            emit attitudeChanged(attitude_degree.pitch, attitude_degree.roll, attitude_degree.yaw);
            break;
        case MAVLINK_MSG_ID_PARAM_VALUE:
            paramValue.param_index = mavlink_msg_param_value_get_param_index(&message);  // get param index
            paramValue.param_value = mavlink_msg_param_value_get_param_value(&message);  // get param value
            emit paramValueChanged(paramValue.param_index, paramValue.param_value);
            break;
        case MAVLINK_MSG_ID_SBUS_CHAN_VALUES:
            sbus_chan_values.ch1 = mavlink_msg_sbus_chan_values_get_ch1(&message);
            sbus_chan_values.ch2 = mavlink_msg_sbus_chan_values_get_ch2(&message);
            sbus_chan_values.ch3 = mavlink_msg_sbus_chan_values_get_ch3(&message);
            sbus_chan_values.ch4 = mavlink_msg_sbus_chan_values_get_ch4(&message);
            sbus_chan_values.ch5 = mavlink_msg_sbus_chan_values_get_ch5(&message);
            sbus_chan_values.ch6 = mavlink_msg_sbus_chan_values_get_ch6(&message);
            sbus_chan_values.ch7 = mavlink_msg_sbus_chan_values_get_ch7(&message);
            sbus_chan_values.ch8 = mavlink_msg_sbus_chan_values_get_ch8(&message);
            sbus_chan_values.ch9 = mavlink_msg_sbus_chan_values_get_ch9(&message);
            sbus_chan_values.ch10 = mavlink_msg_sbus_chan_values_get_ch10(&message);
            sbus_chan_values.ch11 = mavlink_msg_sbus_chan_values_get_ch11(&message);
            sbus_chan_values.ch12 = mavlink_msg_sbus_chan_values_get_ch12(&message);
            sbus_chan_values.ch13 = mavlink_msg_sbus_chan_values_get_ch13(&message);
            sbus_chan_values.ch14 = mavlink_msg_sbus_chan_values_get_ch14(&message);
            sbus_chan_values.ch15 = mavlink_msg_sbus_chan_values_get_ch15(&message);
            sbus_chan_values.ch16 = mavlink_msg_sbus_chan_values_get_ch16(&message);
            sbus_chan_values.ch17 = mavlink_msg_sbus_chan_values_get_ch17(&message);
            sbus_chan_values.ch18 = mavlink_msg_sbus_chan_values_get_ch18(&message);
            emit updateSbusValues();
            break;
        default:
            break;
        } // end of switch
    }
    /* read all params at the first time */
    if(readParams == false){
        readParamsOnBoard();
        readParams = true;
    }

    if(ui->tabWidget->currentIndex() == 3){
        chartTimer->start();
        chartTimerStarted = true;
    }
    else{
        chartTimer->stop();
        chartTimerStarted = false;
    }
}

void MainWindow::heartBeatPulseHandle(bool heartbeat_status){
    static uint8_t temp=0;

    ui->BoardConnectionStatusLabel->show();
    if(heartbeat_status){
        watchdogTimer->setInterval(1000);
        ui->hearbeatPulseLabel->setPixmap(QPixmap::fromImage(imageOn));
        ui->BoardConnectionStatusLabel->setText("Communication ok!");
        ui->BoardConnectionStatusLabel->setStyleSheet("color: rgb(0, 170, 0);"); // green color
    }
    else{
        temp++;
        watchdogTimer->setInterval(220);
        ui->BoardConnectionStatusLabel->setText("Communication fail!");
        ui->BoardConnectionStatusLabel->setStyleSheet("color: rgb(255, 0, 0);"); // red color
        if(temp%2){
            ui->hearbeatPulseLabel->setPixmap(QPixmap::fromImage(imageFail));
        }
        else{
            ui->hearbeatPulseLabel->setPixmap(QPixmap::fromImage(imageOff));
        }
    }
}

void MainWindow::updateAttitude(float pitch, float roll, float yaw){
    ui->pitch_ai_label->setText(QString("%1").arg(pitch,4,'f',2));
    pitch +=90;  // offset 90 degree
    if(pitch<0) pitch+=360;
    ui->pitchCompass->setValue(pitch);

    ui->roll_ai_label->setText(QString("%1").arg(roll, 4,'f',2));
    if(roll < 0) roll+=360;
    roll_ai->setValue(roll);

    ui->yaw_ai_label->setText(QString("%1").arg(yaw ,4,'f',2));
    if(yaw<0) yaw+=360;
    ui->yawCompass->setValue(yaw);
}

/* update param value receive from board */
void MainWindow::updateParamValue(uint8_t index, float value){
    switch(index)
    {
    case PARAM_VERSION:
    {
        //do somethings here;

        uint16_t tram=0, chuc=0, donvi=0, val=0;
        val=value;
        tram = uint16_t(uint16_t(val)/100);
        chuc = uint16_t((val%100)/10);
        donvi = uint16_t((val%100)%10);
        ui->firmwarelabel->setText(QString("%1.%2.%3") .arg(tram) .arg(chuc) .arg(donvi));
        oldParamConfig.version = value;
    }
        break;
    case PARAM_SERIAL_NUMBER:
        //do somethings here;
        oldParamConfig.serialNumber = value;
        break;
    case PARAM_PITCH_P:
        ui->pitch_P->setValue(value);
        oldParamConfig.pitchKp = value;
//        qDebug() << "pitch kp:" << oldParamConfig.pitchKp;
         break;
    case PARAM_PITCH_I:
        ui->pitch_I->setValue(value);
        oldParamConfig.pitchKi = value;
//        qDebug() << "pitch ki:" << oldParamConfig.pitchKi;
         break;
    case PARAM_PITCH_D:
        ui->pitch_D->setValue(value);
        oldParamConfig.pitchKd = value;
//        qDebug() << "pitch kd:" << oldParamConfig.pitchKd;
         break;
    case PARAM_ROLL_P:
        ui->roll_P->setValue(value);
        oldParamConfig.rollKp = value;
//        qDebug() << "roll kp:" << oldParamConfig.rollKp;
         break;
    case PARAM_ROLL_I:
        ui->roll_I->setValue(value);
        oldParamConfig.rollKi = value;
//        qDebug() << "roll ki:" << oldParamConfig.rollKi;
         break;
    case PARAM_ROLL_D:
        ui->roll_D->setValue(value);
        oldParamConfig.rollKd = value;
//        qDebug() << "roll kd:" << oldParamConfig.rollKd;
         break;
    case PARAM_YAW_P:
        ui->yaw_P->setValue(value);
        oldParamConfig.yawKp = value;
//        qDebug() << "yaw kp:" << oldParamConfig.yawKp;
         break;
    case PARAM_YAW_I:
        ui->yaw_I->setValue(value);
        oldParamConfig.yawKi = value;
//        qDebug() << "yaw ki:" << oldParamConfig.yawKi;
         break;
    case PARAM_YAW_D:
        ui->yaw_D->setValue(value);
        oldParamConfig.yawKd = value;
//        qDebug() << "yaw kd:" << oldParamConfig.yawKd;
         break;
    case PARAM_PITCH_POWER:
        ui->pitch_Power->setValue(value);
        oldParamConfig.pitchPower = value;
//        qDebug() << "pitch power:" << oldParamConfig.pitchPower;
         break;
    case PARAM_ROLL_POWER:
        ui->roll_Power->setValue(value);
        oldParamConfig.rollPower = value;
//        qDebug() << "roll power:" << oldParamConfig.rollPower;
         break;
    case PARAM_YAW_POWER:
        ui->yaw_Power->setValue(value);
        oldParamConfig.yawPower = value;
//        qDebug() << "yaw power:" << oldParamConfig.yawPower;
         break;
    case PARAM_PITCH_FOLLOW:
        ui->follow_pitch->setValue(value);
        oldParamConfig.pitchFollow = value;
//        qDebug() << "pitch follow:" << oldParamConfig.pitchFollow;
         break;
    case PARAM_ROLL_FOLLOW:
        ui->follow_roll->setValue(value);
        oldParamConfig.rollFollow = value;
//        qDebug() << "roll follow:" << oldParamConfig.rollFollow;
         break;
    case PARAM_YAW_FOLLOW:
        ui->follow_yaw->setValue(value);
        oldParamConfig.yawFollow = value;
//        qDebug() << "yaw follow:" << oldParamConfig.yawFollow;
         break;
    case PARAM_PITCH_FILTER:
        ui->pitch_filter->setValue(value);
        oldParamConfig.tiltFilter = value;
//        qDebug() << "yaw follow:" << oldParamConfig.yawFollow;
         break;
    case PARAM_ROLL_FILTER:
        ui->roll_filter->setValue(value);
        oldParamConfig.rollFilter = value;
//        qDebug() << "yaw follow:" << oldParamConfig.yawFollow;
         break;
    case PARAM_YAW_FILTER:
        ui->yaw_filter->setValue(value);
        oldParamConfig.panFilter = value;
//        qDebug() << "yaw follow:" << oldParamConfig.yawFollow;
         break;
    case PARAM_GYRO_TRUST:
        ui->gyroTrust->setValue(value);
        oldParamConfig.gyroTrust = value;
//        qDebug() << "gyro trust:" << oldParamConfig.gyroTrust;
        break;
    case PARAM_NPOLES_PITCH:
        ui->pitch_Pole->setValue(value);
        oldParamConfig.nPolesPitch = value;
//        qDebug() << "npoles pitch:" << oldParamConfig.nPolesPitch;
         break;
    case PARAM_NPOLES_ROLL:
        ui->roll_Pole->setValue(value);
        oldParamConfig.nPolesRoll= value;
//        qDebug() << "npoles roll:" << oldParamConfig.nPolesRoll;
         break;
    case PARAM_NPOLES_YAW:
        ui->yaw_Pole->setValue(value);
        oldParamConfig.nPolesYaw = value;
//        qDebug() << "npoles yaw:" << oldParamConfig.nPolesYaw;
        break;
    case PARAM_DIR_MOTOR_PITCH:
        if(value == 0)
            ui->pitch_Dir->setCurrentIndex(0); // normal
        else
            ui->pitch_Dir->setCurrentIndex(1); // reverse
        oldParamConfig.dirMotorPitch = value;
//        qDebug() << "dir motor pitch:" << oldParamConfig.dirMotorPitch;
        break;
    case PARAM_DIR_MOTOR_ROLL:
        if(value == 0)
            ui->roll_Dir->setCurrentIndex(0); // normal
        else
            ui->roll_Dir->setCurrentIndex(1); // reverse
        oldParamConfig.dirMotorRoll = value;
//        qDebug() << "dir motor roll:" << oldParamConfig.dirMotorRoll;
        break;
    case PARAM_DIR_MOTOR_YAW:
        if(value == 0)
            ui->yaw_Dir->setCurrentIndex(0); // normal
        else
            ui->yaw_Dir->setCurrentIndex(1); // reverse
        oldParamConfig.dirMotorYaw = value;
//        qDebug() << "dir motor yaw:" << oldParamConfig.dirMotorYaw;
        break;
    case PARAM_MOTOR_FREQ:
        if(value == 0)
            ui->motor_freq->setCurrentIndex(0); //high
        else if(value == 1)
            ui->motor_freq->setCurrentIndex(1); //medium
        else if(value == 2)
            ui->motor_freq->setCurrentIndex(2); //low
        oldParamConfig.motorFreq = value;
//        qDebug() << "motor freq:" << oldParamConfig.motorFreq;
        break;
    case PARAM_RADIO_TYPE:
        if(value == 0)
            ui->rc_source->setCurrentIndex(0); // sbus
        else if (value == 1)
            ui->rc_source->setCurrentIndex(1); // ppm
        oldParamConfig.radioType = value;
//        qDebug() << "radio type:" << oldParamConfig.radioType;
        break;
    case PARAM_GYRO_LPF:
        ui->gyro_LPF->setValue(value);
        oldParamConfig.gyroLPF = value;
//        qDebug() << "gyro LPF:" << oldParamConfig.gyroLPF;
        break;
    case PARAM_TRAVEL_MIN_PITCH:
        ui->pitch_minTravel->setValue(value);
        oldParamConfig.travelMinPitch = value;
//        qDebug() << "min pitch:" << oldParamConfig.travelMinPitch;
        break;
    case PARAM_TRAVEL_MAX_PITCH:
        ui->pitch_maxTravel->setValue(value);
        oldParamConfig.travelMaxPitch = value;
//        qDebug() << "max pitch:" << oldParamConfig.travelMaxPitch;
        break;
    case PARAM_TRAVEL_MIN_ROLL:
        ui->roll_minTravel->setValue(value);
        oldParamConfig.travelMinRoll = value;
//        qDebug() << "min roll:" << oldParamConfig.travelMinRoll;
        break;
    case PARAM_TRAVEL_MAX_ROLL:
        ui->roll_maxTravel->setValue(value);
        oldParamConfig.travelMaxRoll = value;
//        qDebug() << "max roll:" << oldParamConfig.travelMaxRoll;
        break;
    case PARAM_TRAVEL_MIN_YAW:
        ui->yaw_minTravel->setValue(value);
        oldParamConfig.travelMinYaw = value;
//        qDebug() << "min yaw:" << oldParamConfig.travelMinYaw;
        break;
    case PARAM_TRAVEL_MAX_YAW:
        ui->yaw_maxTravel->setValue(value);
        oldParamConfig.travelMaxYaw = value;
//        qDebug() << "max yaw:" << oldParamConfig.travelMaxYaw;
        break;
    case PARAM_RC_PITCH_LPF:
        ui->rcLPF_pitch->setValue(value);
        oldParamConfig.rcPitchLPF = value;
//        qDebug() << "rc LPF pitch:" << oldParamConfig.rcPitchLPF;
        break;
    case PARAM_RC_ROLL_LPF:
        ui->rcLPF_roll->setValue(value);
        oldParamConfig.rcRollLPF = value;
//        qDebug() << "rc LPF roll:" << oldParamConfig.rcRollLPF;
        break;
    case PARAM_RC_YAW_LPF:
        ui->rcLPF_yaw->setValue(value);
        oldParamConfig.rcYawLPF = value;
//        qDebug() << "rc LPF yaw:" << oldParamConfig.rcYawLPF;
        break;
    case PARAM_SBUS_PITCH_CHAN:
        ui->pitchChan->setCurrentIndex(value);
        oldParamConfig.sbusPitchChan = value;
//        qDebug() << "sbus pitch chan:" << oldParamConfig.sbusPitchChan;
        break;
    case PARAM_SBUS_ROLL_CHAN:
        ui->rollChan->setCurrentIndex(value);
        oldParamConfig.sbusRollChan = value;
//        qDebug() << "sbus roll chan:" << oldParamConfig.sbusRollChan;
        break;
    case PARAM_SBUS_YAW_CHAN:
        ui->yawChan->setCurrentIndex(value);
        oldParamConfig.sbusYawChan = value;
//        qDebug() << "sbus yaw chan:" << oldParamConfig.sbusYawChan;
        break;
    case PARAM_SBUS_MODE_CHAN:
        ui->modeChan->setCurrentIndex(value);
        oldParamConfig.sbusModeChan = value;
//        qDebug() << "sbus mode chan:" << oldParamConfig.sbusModeChan;
        break;
    case PARAM_ACCX_OFFSET:
        ui->accX_offset->setValue(value);
        oldParamConfig.accXOffset = value;
//        qDebug() << "acc X offset:" << oldParamConfig.accXOffset;
        break;
    case PARAM_ACCY_OFFSET:
        ui->accY_offset->setValue(value);
        oldParamConfig.accYOffset = value;
//        qDebug() << "acc Y offset:" << oldParamConfig.accYOffset;
        break;
    case PARAM_ACCZ_OFFSET:
        ui->accZ_offset->setValue(value);
        oldParamConfig.accZOffset = value;
//        qDebug() << "acc Z offset:" << oldParamConfig.accZOffset;
        break;
    case PARAM_USE_GPS:
        ui->useGPS->setChecked(value);
        oldParamConfig.useGPS = value;
//        qDebug() << "use GPS:" << oldParamConfig.useGPS;
        break;
    case PARAM_GYROX_OFFSET:
        ui->gyroX_offset->setValue(value);
        oldParamConfig.gyroXOffset = value;
//        qDebug() << "acc Z offset:" << oldParamConfig.accZOffset;
        break;
    case PARAM_GYROY_OFFSET:
        ui->gyroY_offset->setValue(value);
        oldParamConfig.gyroYOffset = value;
//        qDebug() << "acc Z offset:" << oldParamConfig.accZOffset;
        break;
    case PARAM_GYROZ_OFFSET:
        ui->gyroZ_offset->setValue(value);
        oldParamConfig.gyroZOffset = value;
//        qDebug() << "acc Z offset:" << oldParamConfig.accZOffset;
        break;
    case PARAM_SKIP_GYRO_CALIB:
        ui->calibGyro->setChecked(value);
        oldParamConfig.skipGyroCalib = value;
//        qDebug() << "use GPS:" << oldParamConfig.useGPS;
        break;       
    case PARAM_RC_PITCH_TRIM:
        ui->trim_pitch->setValue(value);
        oldParamConfig.rcPitchTrim = value;
//        qDebug() << "use GPS:" << oldParamConfig.useGPS;
        break;
    case PARAM_RC_ROLL_TRIM:
        ui->trim_roll->setValue(value);
        oldParamConfig.rcRollTrim = value;
//        qDebug() << "use GPS:" << oldParamConfig.useGPS;
        break;
    case PARAM_RC_YAW_TRIM:
        ui->trim_yaw->setValue(value);
        oldParamConfig.rcYawTrim = value;
//        qDebug() << "use GPS:" << oldParamConfig.useGPS;
        break;
    case PARAM_RC_PITCH_MODE:
        ui->mode_pitch->setCurrentIndex(value);
        oldParamConfig.rcPitchMode = value;
//        qDebug() << "use GPS:" << oldParamConfig.useGPS;
        break;
    case PARAM_RC_ROLL_MODE:
        ui->mode_roll->setCurrentIndex(value);
        oldParamConfig.rcRollMode = value;
//        qDebug() << "use GPS:" << oldParamConfig.useGPS;
        break;
    case PARAM_RC_YAW_MODE:
        ui->mode_yaw->setCurrentIndex(value);
        oldParamConfig.rcYawMode = value;
//        qDebug() << "use GPS:" << oldParamConfig.useGPS;
        break;
    default:
        break;
    }
}

void MainWindow::readParamsOnBoard()
{
    uint16_t len;
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    if(serialport->isOpen())
    {
        mavlink_msg_param_request_list_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, MAV_COMP_ID_IMU);
        len = mavlink_msg_to_send_buffer(buf, &msg);
        serialport->write((const char*)buf, len);
    }
}

void MainWindow::writeParamstoBoard(){
    uint16_t len=0;
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];

    if(ui->tabWidget->currentIndex()== 0)  // motor and pid config tag
    {
        /* motor config */
        /* Power percent */
        if(ui->roll_Power->value()!= oldParamConfig.rollPower)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "ROLL_POWER", ui->roll_Power->value(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.rollPower = ui->roll_Power->value();
        }

        if(ui->pitch_Power->value()!= oldParamConfig.pitchPower)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "PITCH_POWER", ui->pitch_Power->value(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.pitchPower = ui->pitch_Power->value();
        }

        if(ui->yaw_Power->value()!= oldParamConfig.yawPower)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "YAW_POWER", ui->yaw_Power->value(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.yawPower = ui->yaw_Power->value();
        }

        /* Number of Poles */
        if(ui->roll_Pole->value()!= oldParamConfig.nPolesRoll)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "NPOLES_ROLL", ui->roll_Pole->value(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.nPolesRoll = ui->roll_Pole->value();
        }

        if(ui->pitch_Pole->value()!= oldParamConfig.nPolesPitch)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "NPOLES_PITCH", ui->pitch_Pole->value(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.nPolesPitch = ui->pitch_Pole->value();
        }

        if(ui->yaw_Pole->value()!= oldParamConfig.nPolesYaw)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "NPOLES_YAW", ui->yaw_Pole->value(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.nPolesYaw =ui->yaw_Pole->value();
        }

        /* Roll Motor limited range travel*/
        if(ui->roll_maxTravel->value()!= oldParamConfig.travelMaxRoll)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "TRAVEL_MAX_ROLL", ui->roll_maxTravel->value(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.travelMaxRoll = ui->roll_maxTravel->value();
        }

        if(ui->roll_minTravel->value()!= oldParamConfig.travelMinRoll)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "TRAVEL_MIN_ROLL", ui->roll_minTravel->value(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.travelMinRoll = ui->roll_minTravel->value();
        }

        /* Pitch Motor limited range travel*/
        if(ui->pitch_maxTravel->value()!= oldParamConfig.travelMaxPitch)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "TRAVEL_MAX_PIT", ui->pitch_maxTravel->value(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.travelMaxPitch = ui->pitch_maxTravel->value();
        }

        if(ui->pitch_minTravel->value()!= oldParamConfig.travelMinPitch)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "TRAVEL_MIN_PIT", ui->pitch_minTravel->value(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.travelMinPitch = ui->pitch_minTravel->value();
        }

        /* Yaw Motor limited range travel*/
        if(ui->yaw_maxTravel->value()!= oldParamConfig.travelMaxYaw)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "TRAVEL_MAX_YAW", ui->yaw_maxTravel->value(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.travelMaxYaw = ui->yaw_maxTravel->value();
        }

        if(ui->yaw_minTravel->value()!= oldParamConfig.travelMinYaw)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "TRAVEL_MIN_YAW", ui->yaw_minTravel->value(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.travelMinYaw = ui->yaw_minTravel->value();
        }

        /* Motor Direction */
        if(ui->roll_Dir->currentIndex() != oldParamConfig.dirMotorRoll)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "DIR_MOTOR_ROLL", ui->roll_Dir->currentIndex(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.dirMotorRoll = ui->roll_Dir->currentIndex();
        }

        if(ui->pitch_Dir->currentIndex() != oldParamConfig.dirMotorPitch)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "DIR_MOTOR_PITCH", ui->pitch_Dir->currentIndex(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.dirMotorPitch = ui->pitch_Dir->currentIndex();
        }

        if(ui->yaw_Dir->currentIndex() != oldParamConfig.dirMotorYaw)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "DIR_MOTOR_YAW", ui->yaw_Dir->currentIndex(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.dirMotorYaw = ui->yaw_Dir->currentIndex();
        }

        /* Motor Frequency */
        if(ui->motor_freq->currentIndex() != oldParamConfig.motorFreq)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "MOTOR_FREQ", ui->motor_freq->currentIndex(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.motorFreq = ui->motor_freq->currentIndex();
        }


        /* pid config */
        /* roll P, I, D */
        if(ui->roll_P->value() != oldParamConfig.rollKp)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "ROLL_P", ui->roll_P->value(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.rollKp = ui->roll_P->value();
        }

        if(ui->roll_I->value() != oldParamConfig.rollKi)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "ROLL_I", ui->roll_I->value(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.rollKi = ui->roll_I->value();
        }

        if(ui->roll_D->value() != oldParamConfig.rollKd)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "ROLL_D", ui->roll_D->value(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.rollKd = ui->roll_D->value();
        }

        /* pitch P, I, D */
        if(ui->pitch_P->value() != oldParamConfig.pitchKp)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "PITCH_P", ui->pitch_P->value(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.pitchKp = ui->pitch_P->value();
        }

        if(ui->pitch_I->value() != oldParamConfig.pitchKi)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "PITCH_I", ui->pitch_I->value(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.pitchKi = ui->pitch_I->value();
        }

        if(ui->pitch_D->value() != oldParamConfig.pitchKd)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "PITCH_D", ui->pitch_D->value(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.pitchKd = ui->pitch_D->value();
        }

        /* yaw P, I, D */
        if(ui->yaw_P->value() != oldParamConfig.yawKp)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "YAW_P", ui->yaw_P->value(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.yawKp = ui->yaw_P->value();
        }

        if(ui->yaw_I->value() != oldParamConfig.yawKi)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "YAW_I", ui->yaw_I->value(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.yawKi = ui->yaw_I->value();
        }

        if(ui->yaw_D->value() != oldParamConfig.yawKd)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "YAW_D", ui->yaw_D->value(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.yawKd = ui->yaw_D->value();
        }

        /* follow mode */
        if(ui->follow_roll->value() != oldParamConfig.rollFollow)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "ROLL_FOLLOW", ui->follow_roll->value(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.rollFollow = ui->follow_roll->value();
        }

        if(ui->follow_pitch->value() != oldParamConfig.pitchFollow)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "PITCH_FOLLOW", ui->follow_pitch->value(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.pitchFollow = ui->follow_pitch->value();
        }

        if(ui->follow_yaw->value() != oldParamConfig.yawFollow)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "YAW_FOLLOW", ui->follow_yaw->value(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.yawFollow = ui->follow_yaw->value();
        }

        if(ui->roll_filter->value() != oldParamConfig.rollFilter)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "ROLL_FILTER", ui->roll_filter->value(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.rollFilter = ui->roll_filter->value();
        }

        if(ui->pitch_filter->value() != oldParamConfig.tiltFilter)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "PITCH_FILTER", ui->pitch_filter->value(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.tiltFilter = ui->pitch_filter->value();
        }

        if(ui->yaw_filter->value() != oldParamConfig.panFilter)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "YAW_FILTER", ui->yaw_filter->value(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.panFilter = ui->yaw_filter->value();
        }
    }
    else if(ui->tabWidget->currentIndex()== 1)  // imu config tab
    {
        /* imu config */
        if(ui->gyroTrust->value() != oldParamConfig.gyroTrust)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "GYRO_TRUST" , ui->gyroTrust->value(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.gyroTrust = ui->gyroTrust->value();
        }

        if(ui->gyro_LPF->value() != oldParamConfig.gyroLPF)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "GYRO_LPF" , ui->gyro_LPF->value(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.gyroLPF = ui->gyro_LPF->value();
        }

        if(ui->accX_offset->value() != oldParamConfig.accXOffset)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "ACCX_OFFSET", ui->accX_offset->value(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.accXOffset = ui->accX_offset->value();
        }

        if(ui->accY_offset->value() != oldParamConfig.accYOffset)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "ACCY_OFFSET", ui->accY_offset->value(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.accYOffset = ui->accY_offset->value();
        }

        if(ui->accZ_offset->value() != oldParamConfig.accZOffset)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "ACCZ_OFFSET", ui->accZ_offset->value(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.accZOffset = ui->accZ_offset->value();
        }

        if(ui->useGPS->isChecked() != oldParamConfig.useGPS)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "USE_GPS", ui->useGPS->isChecked(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.useGPS = ui->useGPS->isChecked();
        }

        if(ui->gyroX_offset->value() != oldParamConfig.gyroXOffset)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "GYROX_OFFSET", ui->gyroX_offset->value(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.gyroXOffset = ui->gyroX_offset->value();
        }

        if(ui->gyroY_offset->value() != oldParamConfig.gyroYOffset)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "GYROY_OFFSET", ui->gyroY_offset->value(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.gyroYOffset = ui->gyroY_offset->value();
        }

        if(ui->gyroZ_offset->value() != oldParamConfig.gyroZOffset)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "GYROZ_OFFSET", ui->gyroZ_offset->value(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.gyroZOffset = ui->gyroZ_offset->value();
        }

        if(ui->calibGyro->isChecked() != oldParamConfig.skipGyroCalib)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "SKIP_GYRO_CALIB", ui->calibGyro->isChecked(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.skipGyroCalib = ui->calibGyro->isChecked();
        }
    }
    else if(ui->tabWidget->currentIndex()== 2)  // rc config tab
    {
        /* RC config */
        /* rc type */
        if(ui->rc_source->currentIndex() != oldParamConfig.radioType)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "RADIO_TYPE", ui->rc_source->currentIndex(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.radioType = ui->rc_source->currentIndex();
        }

        /* channel */
        if(ui->rollChan->currentIndex() != oldParamConfig.sbusRollChan)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "SBUS_ROLL_CHAN", ui->rollChan->currentIndex(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.sbusRollChan = ui->rollChan->currentIndex();
        }

        if(ui->pitchChan->currentIndex() != oldParamConfig.sbusPitchChan)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "SBUS_PITCH_CHAN", ui->pitchChan->currentIndex(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.sbusPitchChan = ui->pitchChan->currentIndex();
        }

        if(ui->yawChan->currentIndex() != oldParamConfig.sbusYawChan)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "SBUS_YAW_CHAN", ui->yawChan->currentIndex(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.sbusYawChan = ui->yawChan->currentIndex();
        }

        if(ui->modeChan->currentIndex() != oldParamConfig.sbusModeChan)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "SBUS_MODE_CHAN", ui->modeChan->currentIndex(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.sbusModeChan = ui->modeChan->currentIndex();
        }

        /* rc LPF*/
        if(ui->rcLPF_roll->value() != oldParamConfig.rcRollLPF)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "RC_ROLL_LPF" , ui->rcLPF_roll->value(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.rcRollLPF = ui->rcLPF_roll->value();
        }

        if(ui->rcLPF_pitch->value() != oldParamConfig.rcPitchLPF)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "RC_PITCH_LPF" , ui->rcLPF_pitch->value(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.rcPitchLPF = ui->rcLPF_pitch->value();
        }

        if(ui->rcLPF_yaw->value() != oldParamConfig.rcYawLPF)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "RC_YAW_LPF" , ui->rcLPF_yaw->value(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.rcYawLPF = ui->rcLPF_yaw->value();
        }

        /* rc trim value */
        if(ui->trim_roll->value() != oldParamConfig.rcRollTrim)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "RC_ROLL_TRIM" , ui->trim_roll->value(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.rcRollTrim = ui->trim_roll->value();
        }

        if(ui->trim_pitch->value() != oldParamConfig.rcPitchTrim)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "RC_PITCH_TRIM" , ui->trim_pitch->value(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.rcPitchTrim = ui->trim_pitch->value();
        }

        if(ui->trim_yaw->value() != oldParamConfig.rcYawTrim)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "RC_YAW_TRIM" , ui->trim_yaw->value(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.rcYawTrim = ui->trim_yaw->value();
        }

        /* rc mode */
        if(ui->mode_roll->currentIndex() != oldParamConfig.rcRollMode)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "RC_ROLL_MODE" , ui->mode_roll->currentIndex(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.rcRollMode = ui->mode_roll->currentIndex();
        }

        if(ui->mode_pitch->currentIndex() != oldParamConfig.rcPitchMode)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "RC_PITCH_MODE" , ui->mode_pitch->currentIndex(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.rcPitchMode = ui->mode_pitch->currentIndex();
        }

        if(ui->mode_yaw->currentIndex() != oldParamConfig.rcYawMode)
        {
            mavlink_msg_param_set_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg, TARGET_SYSTEM_ID, \
                                       MAV_COMP_ID_IMU, "RC_YAW_MODE" , ui->mode_yaw->currentIndex(), MAVLINK_TYPE_INT16_T);
            len = mavlink_msg_to_send_buffer(buf, &msg);
            serialport->write((const char*)buf, len);
            oldParamConfig.rcYawMode = ui->mode_yaw->currentIndex();
        }
    }
    else if(ui->tabWidget->currentIndex()== 3)  // data display tab
    {
        // show message read only
        QMessageBox::information(this, "Message", "You can not write or change these values, just read only!");
    }
}

void MainWindow::timeOutHandle(){
    emit heartBeatPulse(false);
//    int res = QMessageBox::warning(this, "Connection Error", "Board is not found. Please try again!");
//    if(res == QMessageBox::Ok)
      timerRestart();
}

void MainWindow::timerRestart(){
    watchdogTimer->stop();
    watchdogTimer->start();
}

/* set all params to default values */
void MainWindow::setDefaultParams()
{
    /* tab motor config */
    ui->roll_Power->setValue(90);  // 70% of max power
    ui->roll_Pole->setValue(24);   // 24 poles
    ui->roll_maxTravel->setValue(45);  // max travel degree
    ui->roll_minTravel->setValue(-45); // min travle degree
    ui->roll_Dir->setCurrentIndex(1);  // normal direction

    ui->pitch_Power->setValue(80);
    ui->pitch_Pole->setValue(24);
    ui->pitch_maxTravel->setValue(15);
    ui->pitch_minTravel->setValue(-90);
    ui->pitch_Dir->setCurrentIndex(1);

    ui->yaw_Power->setValue(100);
    ui->yaw_Pole->setValue(24);
    ui->yaw_maxTravel->setValue(180);
    ui->yaw_minTravel->setValue(-180);
    ui->yaw_Dir->setCurrentIndex(1);

    ui->motor_freq->setCurrentIndex(1);  // high freq

    /* tab pid config */
    ui->roll_P->setValue(25);
    ui->roll_I->setValue(10);
    ui->roll_D->setValue(5);

    ui->pitch_P->setValue(30);
    ui->pitch_I->setValue(15);
    ui->pitch_D->setValue(5);

    ui->yaw_P->setValue(20);
    ui->yaw_I->setValue(5);
    ui->yaw_D->setValue(4);

    ui->follow_roll->setValue(0); // percent
    ui->follow_pitch->setValue(20);
    ui->follow_yaw->setValue(20);

    ui->roll_filter->setValue(0);
    ui->pitch_filter->setValue(20);
    ui->yaw_filter->setValue(20);

    /* tab rc config */
    ui->rc_source->setCurrentIndex(1);  // ppm
    ui->rcLPF_roll->setValue(1);
    ui->rcLPF_pitch->setValue(1);
    ui->rcLPF_yaw->setValue(1);
    ui->rollChan->setCurrentIndex(7);  //channel 1
    ui->pitchChan->setCurrentIndex(2); //channel 2
    ui->yawChan->setCurrentIndex(0);   //channel 3
    ui->modeChan->setCurrentIndex(5);  //channel 4

    ui->trim_pitch->setValue(0);
    ui->trim_roll->setValue(0);
    ui->trim_yaw->setValue(0);

    ui->mode_pitch->setCurrentIndex(0);
    ui->mode_roll->setCurrentIndex(0);
    ui->mode_yaw->setCurrentIndex(0);

    /* ctab IMU config */
    ui->gyroTrust->setValue(120);
    ui->gyro_LPF->setValue(0);
    ui->accX_offset->setValue(0);
    ui->accY_offset->setValue(700);
    ui->accZ_offset->setValue(0);

    ui->useGPS->setChecked(false);

    ui->gyroX_offset->setValue(0);
    ui->gyroY_offset->setValue(0);
    ui->gyroZ_offset->setValue(0);
    ui->calibGyro->setChecked(false);  
}

void MainWindow::updateSbusValues()
{
    ui->ch1level_Bar->setValue(sbus_chan_values.ch1);
    ui->ch2level_Bar->setValue(sbus_chan_values.ch2);
    ui->ch3level_Bar->setValue(sbus_chan_values.ch3);
    ui->ch4level_Bar->setValue(sbus_chan_values.ch4);
    ui->ch5level_Bar->setValue(sbus_chan_values.ch5);
    ui->ch6level_Bar->setValue(sbus_chan_values.ch6);
    ui->ch7level_Bar->setValue(sbus_chan_values.ch7);
    ui->ch8level_Bar->setValue(sbus_chan_values.ch8);
    ui->ch9level_Bar->setValue(sbus_chan_values.ch9);
    ui->ch10level_Bar->setValue(sbus_chan_values.ch10);
    ui->ch11level_Bar->setValue(sbus_chan_values.ch11);
    ui->ch12level_Bar->setValue(sbus_chan_values.ch12);
    ui->ch13level_Bar->setValue(sbus_chan_values.ch13);
    ui->ch14level_Bar->setValue(sbus_chan_values.ch14);
    ui->ch15level_Bar->setValue(sbus_chan_values.ch15);
    ui->ch16level_Bar->setValue(sbus_chan_values.ch16);
    ui->ch17level_Bar->setValue(sbus_chan_values.ch17);
    ui->ch18level_Bar->setValue(sbus_chan_values.ch18);
}

void MainWindow::chartUpdateData()
{
    ++time_count;

    sampleSize = ax_point.size();

    if(sampleSize >= sampleSizeMax)
    {
        ax_point.clear();
        ay_point.clear();
        sampleSizeOverflow++;
    }

    if((ax_point.size() % interval_value)== 0)
    {
        ui->chartPlot->setAxisScale(QwtPlot::xBottom, (sampleSizeOverflow*sampleSizeMax) + ax_point.size(),
                                    (sampleSizeOverflow*sampleSizeMax) + ax_point.size() + interval_value);
    }

    ax_point += QPointF (time_count, raw_imu.xacc);
    ui->ax_label->setText(QString("%1").arg(raw_imu.xacc, 2));

    ay_point += QPointF (time_count, raw_imu.yacc);
    ui->ay_label->setText(QString("%1").arg(raw_imu.yacc));

    az_point += QPointF (time_count, raw_imu.zacc);
    ui->az_label->setText(QString("%1").arg(raw_imu.zacc));

    gx_point += QPointF (time_count, raw_imu.xgyro);
    ui->gx_label->setText(QString("%1").arg(raw_imu.xgyro));

    gy_point += QPointF (time_count, raw_imu.ygyro);
    ui->gy_label->setText(QString("%1").arg(raw_imu.ygyro));

    gz_point += QPointF (time_count, raw_imu.zgyro);
    ui->gz_label->setText(QString("%1").arg(raw_imu.zgyro));

    pitch_point += QPointF (time_count, attitude_degree.pitch);
    ui->pitch_label->setText(QString("%1").arg(attitude_degree.pitch,4,'f',2));

    roll_point += QPointF (time_count, attitude_degree.roll);
    ui->roll_label->setText(QString("%1").arg(attitude_degree.roll,4,'f',2));

    yaw_point += QPointF (time_count, attitude_degree.yaw);
    ui->yaw_label->setText(QString("%1").arg(attitude_degree.yaw,4,'f',2));

    ax_curve->setSamples(ax_point);
    ay_curve->setSamples(ay_point);
    az_curve->setSamples(az_point);

    gx_curve->setSamples(gx_point);
    gy_curve->setSamples(gy_point);
    gz_curve->setSamples(gz_point);

    pitch_curve->setSamples(pitch_point);
    roll_curve->setSamples(roll_point);
    yaw_curve->setSamples(yaw_point);

    ui->chartPlot->replot();
}

void MainWindow::on_upgradeFWButton_clicked()
{
    QString appFolder = QCoreApplication::applicationDirPath()+ "/thirdParty/FlyMCU.exe" ;
    appFolder.replace("/","\\\\");  // replace "/ to "\\" in order to run th FlyMCU.exe
    qDebug()<< appFolder;

    if(!serialport->isOpen())
    {
        int res = QMessageBox::information(this,"Upgrade Firmware Confirm", "Press OK to Open the Upgrade FW Dialog",QMessageBox::Ok,QMessageBox::Cancel);
        if(res == QMessageBox::Ok)
        {
             m_process.execute(appFolder, QStringList() << "" );
        }
    }
    else
        QMessageBox::information(this,"Message", "Please click Disconnect button first!",QMessageBox::Ok,QMessageBox::Cancel);

}

void MainWindow::on_SerialPortConnectButton_clicked(){
    watchdogTimer->setInterval(3000);
    watchdogTimer->setSingleShot(true);

    chartTimer->setInterval(10);

    openSerialPort();
}

void MainWindow::onSelectPortChanged(const QString &newPortName){
    serialport->flush();
    if(serialport->isOpen())
    {
        serialport->close();
        watchdogTimer->stop();
    }

    serialport->setPortName(newPortName);
    serialport->close();
    m_statusLabel->setText(QString("Port %1 is selected").arg(serialport->portName()));
    updatePortStatus(serialport->isOpen());
}

/* clear all params to zero */
void MainWindow::on_clearParamButton_clicked(){
    /* clear tab motor and pid config */
    ui->roll_Power->setValue(0);
    ui->roll_Pole->setValue(0);
    ui->roll_maxTravel->setValue(0);
    ui->roll_minTravel->setValue(0);
    ui->roll_Dir->setCurrentIndex(2);

    ui->pitch_Power->setValue(0);
    ui->pitch_Pole->setValue(0);
    ui->pitch_maxTravel->setValue(0);
    ui->pitch_minTravel->setValue(0);
    ui->pitch_Dir->setCurrentIndex(2);

    ui->yaw_Power->setValue(0);
    ui->yaw_Pole->setValue(0);
    ui->yaw_maxTravel->setValue(0);
    ui->yaw_minTravel->setValue(0);
    ui->yaw_Dir->setCurrentIndex(2);

    ui->motor_freq->setCurrentIndex(3);

    /* clear pid config */
    ui->roll_P->setValue(0);
    ui->roll_I->setValue(0);
    ui->roll_D->setValue(0);

    ui->pitch_P->setValue(0);
    ui->pitch_I->setValue(0);
    ui->pitch_D->setValue(0);

    ui->yaw_P->setValue(0);
    ui->yaw_I->setValue(0);
    ui->yaw_D->setValue(0);

    ui->follow_roll->setValue(0);
    ui->follow_pitch->setValue(0);
    ui->follow_yaw->setValue(0);

    ui->roll_filter->setValue(0);
    ui->roll_filter->setValue(0);
    ui->roll_filter->setValue(0);

    /* clear tab rc config */
    ui->rc_source->setCurrentIndex(2);
    ui->rcLPF_roll->setValue(0);
    ui->rcLPF_pitch->setValue(0);
    ui->rcLPF_yaw->setValue(0);
    ui->rollChan->setCurrentIndex(18);
    ui->pitchChan->setCurrentIndex(18);
    ui->yawChan->setCurrentIndex(18);
    ui->modeChan->setCurrentIndex(18);

    ui->trim_pitch->setValue(0);
    ui->trim_roll->setValue(0);
    ui->trim_yaw->setValue(0);

    ui->mode_pitch->setCurrentIndex(0);
    ui->mode_roll->setCurrentIndex(0);
    ui->mode_yaw->setCurrentIndex(0);

    /* clear tab IMU config */
    ui->gyroTrust->setValue(0);
    ui->gyro_LPF->setValue(0);
    ui->accX_offset->setValue(0);
    ui->accY_offset->setValue(0);
    ui->accZ_offset->setValue(0);

    ui->useGPS->setChecked(false);

    ui->gyroX_offset->setValue(0);
    ui->gyroY_offset->setValue(0);
    ui->gyroZ_offset->setValue(0);
    ui->calibGyro->setChecked(false);
}

void MainWindow::on_ax_CheckBox_toggled(bool checked)
{
    if(checked) ax_curve->show();
    else ax_curve->hide();
}

void MainWindow::on_ay_CheckBox_toggled(bool checked)
{
    if(checked) ay_curve->show();
    else ay_curve->hide();
}

void MainWindow::on_az_CheckBox_toggled(bool checked)
{
    if(checked) az_curve->show();
    else az_curve->hide();
}

void MainWindow::on_gx_CheckBox_toggled(bool checked)
{
    if(checked) gx_curve->show();
    else gx_curve->hide();
}

void MainWindow::on_gy_CheckBox_toggled(bool checked)
{
    if(checked) gy_curve->show();
    else gy_curve->hide();
}

void MainWindow::on_gz_CheckBox_toggled(bool checked)
{
    if(checked) gz_curve->show();
    else gz_curve->hide();
}

void MainWindow::on_pitch_CheckBox_toggled(bool checked)
{
    if(checked) pitch_curve->show();
    else pitch_curve->hide();
}

void MainWindow::on_roll_CheckBox_toggled(bool checked)
{
    if(checked) roll_curve->show();
    else roll_curve->hide();
}

void MainWindow::on_yaw_CheckBox_toggled(bool checked)
{
    if(checked) yaw_curve->show();
    else yaw_curve->hide();
}

void MainWindow::on_pitchSlider_ValueChanged()
{
    uint16_t len=0;
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    int temp=0;
    if(ui->pitchSlider->value() > 0)
        temp = ui->pitchSlider->value() * 10;
    else temp = ui->pitchSlider->value() * 5;

    mavlink_msg_tilt_simulation_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg,
                                     temp, ui->pitchChan->currentIndex());
    qDebug() <<"pitch slider" << temp;
    len = mavlink_msg_to_send_buffer(buf, &msg);
    serialport->write((const char*)buf, len);
}

void MainWindow::on_rollSlider_ValueChanged()
{
    uint16_t len=0;
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    int temp=0;
    temp = ui->rollSlider->value() * 7;

//    if(ui->pitchSlider->value() > 0)
//        temp = ui->pitchSlider->value() * 10;
//    else temp = ui->pitchSlider->value() * 5;


    mavlink_msg_roll_simulation_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg,
                                     temp, ui->rollChan->currentIndex());
    qDebug() <<"roll slider" << temp;
    len = mavlink_msg_to_send_buffer(buf, &msg);
    serialport->write((const char*)buf, len);
}

void MainWindow::on_yawknob_ValueChanged()
{
    uint16_t len=0;
    mavlink_message_t msg;
    uint8_t buf[MAVLINK_MAX_PACKET_LEN];
    int temp=0;
    temp = ui->yawknob->value() * 3;

    mavlink_msg_pan_simulation_pack(SYSTEM_ID, MAV_COMP_ID_SERVO1, &msg,
                                     temp, ui->yawChan->currentIndex());
    qDebug() <<"yaw knob" << temp;
    len = mavlink_msg_to_send_buffer(buf, &msg);
    serialport->write((const char*)buf, len);
}

void MainWindow::on_yawknob_Released()
{
    ui->yawknob->setValue(0);
}

void MainWindow::on_loadfileButtonClicked()
{
    int char_pos=0, filename_len=0;
    int temp=0;
    QString filename;

    loadfilename = QFileDialog::getOpenFileName(this, tr("Open profile"),
                                            "profiles", tr("XML files (*.xml)"));
    qDebug() << loadfilename;
    filename_len = loadfilename.size();
    temp = filename_len;

    while(loadfilename.at(--temp) != '/')
        char_pos = temp;
    filename = loadfilename.right(filename_len - char_pos);
    qDebug() << "file name: " + filename;

    ui->profilename->setText(filename);
    oldprofilename = filename.left(filename.size() - 4);
    qDebug() << "old file name: " + oldprofilename;

    importXMLfile(loadfilename);
}

void MainWindow::on_savefileButtonClicked()
{
    bool dot=false;
    int len_str=0;
    QString str, name_str;

    str = ui->profilename->text();

    len_str = str.size();

    for(int i=0; i<len_str; i++)
    {
        if(str.at(i) == '.')
            dot = true;
    }

    if(dot)
        name_str = str.left(len_str - 4);
    else
        name_str = str;

    if(name_str != oldprofilename)
    {
        oldprofilename = name_str;
        exportXMLfile(name_str + ".xml");
    }
    else
    {
        saveXMLfile(oldprofilename + ".xml");
    }
}

void MainWindow::init_var()
{
    time_count = 0;
    sampleSize = 0;
    sampleSizeOverflow = 0;
    interval_value = (100);
    sampleSizeMax = (1000);
    // Set Image
    imageOn.load(":/files/images/leds/circle_green.svg");
    imageOff.load(":/files/images/leds/circle_black.svg");
    imageFail.load(":/files/images/leds/circle_red.svg");
    // Set Images
    // load Stylesheet
    currentStyle = G_MAINWINDOW_STYLE_INDOOR;
    styleFileName = QCoreApplication::applicationDirPath() + ":general/files/styles/style-indoor.css";

}

QPalette MainWindow::colorTheme(const QColor &base) const
{
    QPalette palette;
    palette.setColor( QPalette::Base, base );
    palette.setColor( QPalette::Window, base.dark( 150 ) );
    palette.setColor( QPalette::Mid, base.dark( 110 ) );
    palette.setColor( QPalette::Light, base.light( 170 ) );
    palette.setColor( QPalette::Dark, base.dark( 170 ) );
    palette.setColor( QPalette::Text, base.dark( 200 ).light( 800 ) );
    palette.setColor( QPalette::WindowText, base.dark( 200 ) );

    return palette;
}
