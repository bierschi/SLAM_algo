/*!
 * @file
 * */
#include "Settings.h"
#include "ui_Settings.h"

#include <QProcess>
#include <QDebug>


Settings::Settings(QMainWindow *parent, QString IP, QString Port, QString Dev) : QDialog(parent),
    settings_ui(new Ui::Settings)
    {

    settings_ui->setupUi(this);

    settings_ui->ip_lineEdit->setText(IP);
    settings_ui->port_lineEdit->setText(Port);

    QProcess process;
    process.start("ls /dev/input/");
    process.waitForFinished(-1);

    QString stdout = process.readAllStandardOutput();

    bool search = true;
    int index_first = 0;
    int index_last = 0;
    qDebug()<<stdout;
    while(search) {
        index_first = stdout.indexOf("js", index_first+1);
        if(index_first != -1) {
            index_last = stdout.indexOf("\n", index_first);
            settings_ui->contr_comboBox->addItem(stdout.midRef(index_first, index_last-index_first).toString());
        }
        else {
            search = false;
        }
    }

    settings_ui->contr_comboBox->setCurrentText(Dev);
}
Settings::~Settings() {

}

void Settings::open() {
    this->show();
}
/*
void Settings::setIP(QString IP) {
    settings_ui->ip_textEdit->setText(IP);
}

void Settings::setPort(QString Port) {
    settings_ui->port_TextEdit->setText(Port);
}

void Settings::getIP(QString *IP) {
    IP = settings_ui->ip_TextEdit->text();
}

void Settings::getPort(QString *Port) {
    Port = settings_ui->port_lineEdit->text();
}
*/
