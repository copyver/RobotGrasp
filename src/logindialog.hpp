//
// Created by yhlever on 25-3-5.
//

#ifndef ROBOT_GRASP_LOGINDIALOG_HPP
#define ROBOT_GRASP_LOGINDIALOG_HPP

#include <QDialog>
#include <QLineEdit>
#include <QPushButton>
#include <QLabel>

class LoginDialog : public QDialog {
    Q_OBJECT
public:
    explicit LoginDialog(QWidget* parent = nullptr);
    QString username() const;

private Q_SLOTS:
    void attemptLogin();

private:
    QLineEdit* m_usernameLineEdit;
    QLineEdit* m_passwordLineEdit;
    QPushButton* m_loginButton;
    QLabel* m_statusLabel;
};



#endif //ROBOT_GRASP_LOGINDIALOG_HPP
