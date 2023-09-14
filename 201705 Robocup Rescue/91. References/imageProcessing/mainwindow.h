#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <QMainWindow>
#include <opencv2/core/core.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/calib3d/calib3d.hpp>

namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);

    int pictureCompare(std::string imgF, std::string imgS, int checking);

    void readme();

    ~MainWindow();

private slots:
    void on_pushButtonShutDown_clicked();

private:
    Ui::MainWindow *ui;
};

#endif // MAINWINDOW_H
