#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "uas/uasmanager.h"
#include "comm/linkmanager.h"

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
    dialog = new SerialSettingDialog;
    connect(ui->actionConfig,SIGNAL(triggered(bool)),dialog,SLOT(show()));

    LinkManager::instance();
    UASManager::instance();



}

MainWindow::~MainWindow()
{
    delete ui;
}
