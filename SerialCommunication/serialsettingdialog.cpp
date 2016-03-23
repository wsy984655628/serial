#include "serialsettingdialog.h"
#include "ui_serialsettingdialog.h"
#include <QtSerialPort/QSerialPortInfo>
#include <QMessageBox>
#include <QLineEdit>
#include <QTextEdit>
#include <QDebug>

#include "comm/linkmanager.h"

static const char blankString[] = QT_TRANSLATE_NOOP("SerialSettingDialog","N/A");

SerialSettingDialog::SerialSettingDialog(QWidget *parent) :
    QDialog(parent),
    ui(new Ui::SerialSettingDialog)
{
    ui->setupUi(this);
    setWindowTitle("Configure Serial Port");
    serial = new QSerialPort(this);
    intValidator = new QIntValidator(0, 4000000, this);

    ui->BaudRate_Box->setInsertPolicy(QComboBox::NoInsert);

    initConnections();

    fillPortsInfo();
    fillPortsParameters();

    updateSettings();

    console = new SerialConsole;
    console->setEnabled(false);
    ui->horizontalLayout->addWidget(console);
}

SerialSettingDialog::~SerialSettingDialog()
{
    delete ui;
}

SerialSettingDialog::Settings SerialSettingDialog::settings() const
{
    return currentSettings;
}

void SerialSettingDialog::showPortInfo(int idx)
{
    if(idx == -1)
        return;

    QStringList list = ui->SerialPort_Box->itemData(idx).toStringList();
    ui->Description_label->setText(tr("Description: %1").arg(list.count() > 1 ? list.at(1) : tr(blankString)));
    ui->Manufacturer_label->setText(tr("Manufacturer: %1").arg(list.count() > 2 ? list.at(2) : tr(blankString)));
    ui->SerialNumber_label->setText(tr("SerialNumber: %1").arg(list.count() > 3 ? list.at(3) : tr(blankString)));
    ui->Location_label->setText(tr("Location: %1").arg(list.count() > 4 ? list.at(4) : tr(blankString)));
    ui->VendorID_label->setText(tr("VendorID: %1").arg(list.count() > 5 ? list.at(5) : tr(blankString)));
    ui->ProductID_label->setText(tr("ProductID: %1").arg(list.count() > 6 ? list.at(6) : tr(blankString)));
}

void SerialSettingDialog::apply()
{
    updateSettings();
    hide();
}

void SerialSettingDialog::checkCustomBaudRatePolicy(int idx)
{
    bool isCustomBaudRate = !ui->BaudRate_Box->itemData(idx).isValid();
    ui->BaudRate_Box->setEditable(isCustomBaudRate);
    if(isCustomBaudRate)
    {
        ui->BaudRate_Box->clearEditText();
        QLineEdit *edit = ui->BaudRate_Box->lineEdit();
        edit->setValidator(intValidator);
    }
}

void SerialSettingDialog::checkCustomDevicePathPolicy(int idx)
{
    bool isCustomPath = !ui->SerialPort_Box->itemData(idx).isValid();
    ui->SerialPort_Box->setEditable(isCustomPath);
    if(isCustomPath)
    {
        ui->SerialPort_Box->clearEditText();
    }
}

void SerialSettingDialog::fillPortsParameters()
{
    ui->BaudRate_Box->addItem(QStringLiteral("9600"), QSerialPort::Baud9600);
    ui->BaudRate_Box->addItem(QStringLiteral("19200"), QSerialPort::Baud19200);
    ui->BaudRate_Box->addItem(QStringLiteral("19200"), QSerialPort::Baud57600);
    ui->BaudRate_Box->addItem(QStringLiteral("115200"), QSerialPort::Baud115200);
    ui->BaudRate_Box->addItem(tr("Custom"));

    ui->DataBits_Box->addItem(QStringLiteral("5"), QSerialPort::Data5);
    ui->DataBits_Box->addItem(QStringLiteral("6"), QSerialPort::Data6);
    ui->DataBits_Box->addItem(QStringLiteral("7"), QSerialPort::Data7);
    ui->DataBits_Box->addItem(QStringLiteral("8"), QSerialPort::Data8);
    ui->DataBits_Box->setCurrentIndex(3);

    ui->Parity_Box->addItem(tr("None"), QSerialPort::NoParity);
    ui->Parity_Box->addItem(tr("Even"), QSerialPort::EvenParity);
    ui->Parity_Box->addItem(tr("Odd"), QSerialPort::OddParity);
    ui->Parity_Box->addItem(tr("Mark"), QSerialPort::MarkParity);
    ui->Parity_Box->addItem(tr("Space"), QSerialPort::SpaceParity);

    ui->StopBits_Box->addItem(QStringLiteral("1"), QSerialPort::OneStop);
    ui->StopBits_Box->addItem(QStringLiteral("2"), QSerialPort::TwoStop);

    ui->FlowControl_Box->addItem(tr("None"), QSerialPort::NoFlowControl);
    ui->FlowControl_Box->addItem(tr("RTS/CTS"), QSerialPort::HardwareControl);
    ui->FlowControl_Box->addItem(tr("XON/XOFF"), QSerialPort::SoftwareControl);
}

void SerialSettingDialog::fillPortsInfo()
{
    ui->SerialPort_Box->clear();
    QString description;
    QString manufacturer;
    QString serialNumber;
    foreach(const QSerialPortInfo &info, QSerialPortInfo::availablePorts()){
        QStringList list;
        description = info.description();
        manufacturer = info.manufacturer();
        serialNumber = info.serialNumber();
        list << info.portName()
             << (!description.isEmpty() ? description : blankString)
             << (!manufacturer.isEmpty() ? manufacturer : blankString)
             << (!serialNumber.isEmpty() ? serialNumber : blankString)
             << info.systemLocation()
             << (info.vendorIdentifier() ? QString::number(info.vendorIdentifier(), 16) : blankString)
             << (info.productIdentifier() ? QString::number(info.productIdentifier(), 16) :blankString);

        ui->SerialPort_Box->addItem(list.first(),list);
    }
}

void SerialSettingDialog::updateSettings()
{
    currentSettings.name = ui->SerialPort_Box->currentText();

    if(ui->BaudRate_Box->currentIndex() == 4)
    {
        currentSettings.baudRate = ui->BaudRate_Box->currentText().toInt();
    }else {
        currentSettings.baudRate = static_cast<QSerialPort::BaudRate>(
                    ui->BaudRate_Box->itemData(ui->BaudRate_Box->currentIndex()).toInt());
    }
    currentSettings.stringBaudRate = QString::number(currentSettings.baudRate);

    currentSettings.dataBits = static_cast<QSerialPort::DataBits>(
                ui->DataBits_Box->itemData(ui->DataBits_Box->currentIndex()).toInt());
    currentSettings.stringDataBits = ui->DataBits_Box->currentText();

    currentSettings.parity = static_cast<QSerialPort::Parity>(
                ui->Parity_Box->itemData(ui->Parity_Box->currentIndex()).toInt());
    currentSettings.stringParity = ui->Parity_Box->currentText();

    currentSettings.stopBits = static_cast<QSerialPort::StopBits>(
                ui->StopBits_Box->itemData(ui->StopBits_Box->currentIndex()).toInt());
    currentSettings.stringStopBits = ui->StopBits_Box->currentText();

    currentSettings.flowControl = static_cast<QSerialPort::FlowControl>(
                ui->FlowControl_Box->itemData(ui->FlowControl_Box->currentIndex()).toInt());
    currentSettings.stringFlowControl = ui->FlowControl_Box->currentText();

    qDebug() << "updata Settings";
}

void SerialSettingDialog::initConnections()
{
    connect(ui->Apply_Button,SIGNAL(clicked(bool)),
            this,SLOT(apply()));
    connect(ui->SerialPort_Box,SIGNAL(currentIndexChanged(int)),
            this,SLOT(showPortInfo(int)));
    connect(ui->BaudRate_Box,SIGNAL(currentIndexChanged(int)),
            this,SLOT(checkCustomBaudRatePolicy(int)));
    connect(ui->SerialPort_Box,SIGNAL(currentIndexChanged(int)),
            this,SLOT(checkCustomDevicePathPolicy(int)));

    connect(serial, SIGNAL(error(QSerialPort::SerialPortError)), this,
            SLOT(handleError(QSerialPort::SerialPortError)));
    connect(serial, SIGNAL(readyRead()), this, SLOT(readData()));
    connect(ui->SendData_Button, SIGNAL(clicked(bool)), this,
            SLOT(SendData()));
    connect(ui->Connect_Button, SIGNAL(clicked(bool)), this,
            SLOT(openSerial()));
    connect(ui->DisConnect_Button, SIGNAL(clicked(bool)), this,
            SLOT(closeSerial()));

    connect(this,SIGNAL(bytesReceived(QByteArray)), LinkManager::instance()->getProtocol(), SLOT(receiveBytes(QByteArray)));
}

/*~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~*/
//handle serial port
void SerialSettingDialog::openSerial()
{
    updateSettings();

    serial->setPortName(currentSettings.name);
    serial->setBaudRate(currentSettings.baudRate);
    serial->setDataBits(currentSettings.dataBits);
    serial->setParity(currentSettings.parity);
    serial->setStopBits(currentSettings.stopBits);
    serial->setFlowControl(currentSettings.flowControl);
    if(serial->open(QIODevice::ReadWrite))
    {
        ui->ConnectState_label->setText("ConnectState: Connect Success");
        qDebug() << "Connect Success";
        qDebug() << tr("Connected to %1 : %2, %3, %4, %5, %6")
                    .arg(currentSettings.name).arg(currentSettings.stringBaudRate).arg(currentSettings.stringDataBits)
                    .arg(currentSettings.stringParity).arg(currentSettings.stringStopBits).arg(currentSettings.stringFlowControl);
    }
    else{
        ui->ConnectState_label->setText("ConnectState: Connect Failed");
    }
}

void SerialSettingDialog::closeSerial()
{
    if(serial->isOpen())
    {
        serial->close();
        ui->ConnectState_label->setText("ConnectState: DisConnect");
    }
}

void SerialSettingDialog::writeData(const QByteArray &data)
{
    serial->write(data);
}

void SerialSettingDialog::handleError(QSerialPort::SerialPortError error)
{
    if (error == QSerialPort::ResourceError){
        QMessageBox::critical(this, tr("Critical Error"), serial->errorString());
        closeSerial();
    }
}

void SerialSettingDialog::SendData()
{
    QByteArray write_Data;
    QString content;
    content = ui->textEdit->toPlainText();
    write_Data = content.toLatin1();
    writeData(write_Data);
}

void SerialSettingDialog::readData()
{
    QByteArray data = serial->readAll();
//    console->putData(data);

    emit bytesReceived(data);
}
