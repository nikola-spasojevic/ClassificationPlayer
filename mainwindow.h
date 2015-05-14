#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <player.h>
#include <mousetracker.h>
#include <helperfunctions.h>
#include <QtGui>


namespace Ui {
class MainWindow;
}

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(QWidget *parent = 0);
    ~MainWindow();

private:
    Ui::MainWindow *ui;
    Player* myPlayer;
    vector< vector<KeyPoint> >  featureVec;

    QPoint prevPoint;
    QPoint topLeftCorner;
    QPoint bottomRightCorner;
    QPixmap px, pxBuffer;
    Rect roi;

    void processROI(Mat roi);
    void connectedComponents(Mat roi);
    void getCascade(Mat roi);

signals:

private slots:
    /**************** FRAME PROCESSING ****************/
    void updatePlayerUI(QImage img);
    void processedPlayerUI(QImage processedImg);
    /**************** FRAME PROCESSING ****************/

    /**************** PLAYER ****************/
    void on_LdBtn_clicked();
    void on_PlyBtn_clicked();
    QString getFormattedTime(int timeInSeconds);
    void on_horizontalSlider_sliderPressed();
    void on_horizontalSlider_sliderReleased();
    void on_horizontalSlider_sliderMoved(int position);
    void on_ffwdBtn_pressed();
    void on_ffwdBtn_released();
    /**************** PLAYER ****************/

    /**************** MOUSE TRACKER ****************/
    void Mouse_current_pos();
    void Mouse_pressed();
    void Mouse_left();
    void Mouse_released();
    /**************** MOUSE TRACKER ****************/
};

#endif // MAINWINDOW_H
