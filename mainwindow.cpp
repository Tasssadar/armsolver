#include "mainwindow.h"
#include "ui_mainwindow.h"

#include <math.h>
#include "RBControl_arm.hpp"

using namespace rb;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    ui->setupUi(this);
}

MainWindow::~MainWindow()
{
    delete ui;
}
