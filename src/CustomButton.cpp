#include "CustomButton.hpp"
#include <QPainter>
#include <QMouseEvent>

SwitchButton::SwitchButton(QWidget *parent) : QWidget(parent), animation(new QPropertyAnimation(this, "offset")) {
    setFixedSize(100, 40);
    animation->setDuration(200);
    animation->setEasingCurve(QEasingCurve::InOutQuad);
}

void SwitchButton::setChecked(bool checkedState) {
    if (checked != checkedState) {
        checked = checkedState;
        Q_EMIT toggled(checked);

        // Animate offset
        int start = m_offset;
        int end = checked ? width() - height() + 2 : 2;
        animation->stop();
        animation->setStartValue(start);
        animation->setEndValue(end);
        animation->start();
    }
}

bool SwitchButton::isChecked() const {
    return checked;
}

int SwitchButton::offset() const {
    return m_offset;
}

void SwitchButton::setOffset(int value) {
    m_offset = value;
    update();
}

void SwitchButton::mousePressEvent(QMouseEvent *) {
    setChecked(!checked);
}


void SwitchButton::paintEvent(QPaintEvent *) {
    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing);

    QColor bgColor = checked ? QColor("#27AE60") : QColor("#BDC3C7");
    painter.setBrush(bgColor);
    painter.setPen(Qt::NoPen);
    painter.drawRoundedRect(rect(), height() / 2, height() / 2);

    QRect handleRect(m_offset, 2, height() - 4, height() - 4);
    painter.setBrush(Qt::white);
    painter.drawEllipse(handleRect);

    painter.setPen(Qt::white);
    painter.setFont(QFont("Arial", 12, QFont::Bold));
    QString text = checked ? "已连接" : "未连接";

    QRect textRect;
    if (checked) {
        textRect = QRect(4, 0, width() - height(), height());
        painter.drawText(textRect, Qt::AlignVCenter | Qt::AlignLeft, text);
    } else {
        textRect = QRect(height(), 0, width() - height() - 4, height());
        painter.drawText(textRect, Qt::AlignVCenter | Qt::AlignRight, text);
    }
}


