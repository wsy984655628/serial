#include "serialconsole.h"
#include <QScrollBar>
#include <QtCore/QDebug>

SerialConsole::SerialConsole(QWidget *parent)
    : QTextEdit(parent)
{
    QPalette p = palette();
    p.setColor(QPalette::Base, Qt::black);
    p.setColor(QPalette::Text, Qt::green);
    setPalette(p);
}

void SerialConsole::putData(const QByteArray &data)
{
    insertPlainText(QString(data));

    QScrollBar *bar = verticalScrollBar();
    bar->setValue(bar->maximum());
}

