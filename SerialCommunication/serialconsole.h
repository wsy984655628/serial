#ifndef SERIALCONSOLE_H
#define SERIALCONSOLE_H

#include <QTextEdit>
#include <QPlainTextEdit>
class SerialConsole : public QTextEdit
{
    Q_OBJECT

public:
    explicit SerialConsole(QWidget *parent = 0);

    void putData(const QByteArray &data);

signals:
    void getData(const QByteArray &data);

};

#endif // SERIALCONSOLE_H
