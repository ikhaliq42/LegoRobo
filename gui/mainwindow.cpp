#include "mainwindow.h"
#include "ui_mainwindow.h"

#include "../controls/sample.c"

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

void MainWindow::on_pushButton_clicked()
{
    main2();
    if (ui->label->text() == "Hi") {
        ui->label->setText("Bye");
    } else {
        ui->label->setText("Hi");
    }
}
