//
// Created by yhlever on 25-4-3.
//

#ifndef ROBOT_GRASP_CUSTOMBUTTON_HPP
#define ROBOT_GRASP_CUSTOMBUTTON_HPP

#include <QWidget>
#include <QPropertyAnimation>

class SwitchButton : public QWidget {
Q_OBJECT
    Q_PROPERTY(int offset READ offset WRITE setOffset)

public:
    explicit SwitchButton(QWidget *parent = nullptr);
    void setChecked(bool checked);
    bool isChecked() const;

    int offset() const;
    void setOffset(int value);

Q_SIGNALS:
    void toggled(bool checked);

protected:
    void paintEvent(QPaintEvent *event) override;
    void mousePressEvent(QMouseEvent *event) override;

private:
    bool checked = false;
    int m_offset = 0;
    QPropertyAnimation *animation;
};



#endif //ROBOT_GRASP_CUSTOMBUTTON_HPP
