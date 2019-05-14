#ifndef ARMWIDGET_H
#define ARMWIDGET_H

#include <QWidget>
#include <memory>


#include <RBControl_arm.hpp>

class ArmWidget : public QWidget
{
    Q_OBJECT
public:
    explicit ArmWidget(QWidget *parent = nullptr);

signals:

public slots:

protected:
    void resizeEvent(QResizeEvent *ev);
    void mousePressEvent(QMouseEvent *ev);
    void mouseReleaseEvent(QMouseEvent *ev);
    void mouseMoveEvent(QMouseEvent *ev);

    void paintEvent(QPaintEvent *event);

private:
    bool m_touched;
    double m_unit;
    QPoint m_cursor;
    QPoint m_origin;
    std::unique_ptr<rb::Arm> m_arm;
};

#endif // ARMWIDGET_H
