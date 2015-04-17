#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <player.h>

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
    vector< vector<KeyPoint> > featureVec;

signals:
    void onFFWpressed();
    void onFFWreleased();

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
    /**************** PLAYER ****************/
    void on_ffwdBtn_pressed();
    void on_ffwdBtn_released();
};

#endif // MAINWINDOW_H
