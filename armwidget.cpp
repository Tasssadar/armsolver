#include "armwidget.h"
#include <QPainter>
#include <QMouseEvent>
#include "RBControl_arm.hpp"

using namespace  rb;

ArmWidget::ArmWidget(QWidget *parent) : QWidget(parent), m_touched(false)
{
    ArmBuilder builder;
    builder.body(60, 110).armOffset(0, 20);

    auto b0 = builder.bone(110);
    b0.relStops(-95_deg, 0_deg);
    b0.calcServoAng([](Angle absAngle, Angle) -> Angle {
        return Angle::PI - (absAngle * -1) + 30_deg;
    });

    auto b1 = builder.bone(140);
    b1.relStops(30_deg, 165_deg)
        .absStops(-20_deg, Angle::PI)
        .baseRelStops(40_deg, 160_deg);
    b1.calcServoAng([](Angle absAngle, Angle) -> Angle {
        absAngle = Arm::clamp(absAngle + Angle::PI*1.5);
        return Angle::PI - (absAngle * -1) + 25_deg;
    });

    m_arm = builder.build();
}

void ArmWidget::resizeEvent(QResizeEvent *) {
    const auto& def = m_arm->definition();
    Arm::CoordType total_len = 0;
    for(const auto& b : def.bones) {
        total_len = b.length;
    }

    m_unit = std::min(width()*0.3, height()*0.3) / total_len;

    m_origin.setX(def.body_radius * m_unit);
    m_origin.setY(height() - (def.body_height*1.3 - def.arm_offset_y)*m_unit);
}

void ArmWidget::mousePressEvent(QMouseEvent *ev) {
    m_cursor = ev->pos();
    m_touched = true;
    this->repaint();
}

void ArmWidget::mouseReleaseEvent(QMouseEvent *ev) {
    m_cursor = ev->pos();
    m_touched = false;
    this->repaint();
}

void ArmWidget::mouseMoveEvent(QMouseEvent *ev) {
    m_cursor = ev->pos();
    this->repaint();
}

void ArmWidget::paintEvent(QPaintEvent *) {
    QPoint target = m_cursor - m_origin;
    target /= m_unit;
    m_arm->solve(target.x(), target.y());

    const auto& def = m_arm->definition();

    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing);


    int y = 20;
    int step = width() / 7;
    QString text;
    p.drawText(0, y, "angle");
    p.drawText(step, y, "dangle");
    p.drawText(step*2, y, "rel");
    p.drawText(step*3, y, "drel");
    p.drawText(step*4, y, "srv");
    p.drawText(step*5, y, "dsrv");
    y+= 15;
    for(const auto& b : m_arm->bones()) {
        int x = 0;
        p.drawText(x, y, QString::asprintf("%8.3f", b.absAngle.rad()));
        x += step;
        p.drawText(x, y, QString::asprintf("%8.3f", b.absAngle.deg()));
        x += step;
        p.drawText(x, y, QString::asprintf("%8.3f", b.relAngle.rad()));
        x += step;
        p.drawText(x, y, QString::asprintf("%8.3f", b.relAngle.deg()));
        x += step;
        p.drawText(x, y, QString::asprintf("%8.3f", b.servoAng().rad()));
        x += step;
        p.drawText(x, y, QString::asprintf("%8.3f", b.servoAng().deg()));

        y += 15;
    }

    y += 15;
    p.drawText(y, y, QString::asprintf("%4d %4d", target.x(), target.y()));


    p.translate(m_origin);
    p.fillRect(target.x()-5, target.y()-5, 10, 10, Qt::blue);

    auto w = def.body_radius*2*m_unit;
    auto h = def.body_height*m_unit;

    p.fillRect(-w/2, def.arm_offset_y*m_unit, w, h, QColor("#C8A165"));

    QPen pen(Qt::black);
    pen.setWidth(2);
    p.setPen(pen);
    for(const auto& b : m_arm->bones()) {
        p.save();
        p.rotate(b.relAngle.deg());

        p.fillRect(-5, -5, 10, 10, Qt::red);
        p.drawLine(0, 0, b.def.length*m_unit, 0);
        p.translate(b.def.length*m_unit, 0);
    }

    for(auto _ : m_arm->bones())
        p.restore();
}
