#include <QApplication>
#include <QFile>
#include <QDebug>
#include "src/logindialog.hpp"
#include "src/mainwindow.hpp"

int main(int argc, char *argv[])
{
    QApplication app(argc, argv);
    QApplication::setStyle("Fusion");


    QFile qssFile(":/theme.qss");
    if (!qssFile.open(QIODevice::ReadOnly | QIODevice::Text)) {
        qDebug() << "Failed to load QSS file:" << qssFile.errorString();
    }
    else {
        app.setStyleSheet(QString::fromUtf8(qssFile.readAll()));
    }
    qssFile.close();

    qRegisterMetaType<std::shared_ptr<PercipioCamera>>("std::shared_ptr<PercipioCamera>");

    LoginDialog loginDialog;
    if (loginDialog.exec() == QDialog::Accepted) {
        MainWindow mainWindow;
        mainWindow.setWindowTitle("机器人抓取系统");
        mainWindow.resize(800, 600);
        mainWindow.show();
        return QApplication::exec();
    }
}
