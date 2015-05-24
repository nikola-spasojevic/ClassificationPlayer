#ifndef MAINWINDOW_H
#define MAINWINDOW_H

#include <player.h>
#include <mousetracker.h>
#include <helperfunctions.h>
#include <QtGui>
#include <opencv2/ml/ml.hpp>

#define DICTIONARY_SIZE 1000
#define MIN_FEATURE_SIZE 7000
#define COUNTOUR_AREA_THRESHOLD 3000

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


    vector<Mat> histogram_sceneVector;
    QPoint prevPoint;
    QPoint topLeftCorner;
    QPoint bottomRightCorner;
    QPixmap px, pxBuffer;
    cv::Rect roi;
    cv::Rect window;
    Ptr<FeatureDetector> detector = new PyramidAdaptedFeatureDetector(new DynamicAdaptedFeatureDetector (new FastAdjuster(100,true), 100, 200, 2), 4);
    Ptr<DescriptorExtractor> extractor = DescriptorExtractor::create("SURF");
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("FlannBased");
    BOWImgDescriptorExtractor bowDE; 
    bool isDictionarySet;

    void processROI(Mat roi);
    Mat connectedComponents(Mat roi);

signals:

private slots:
    /**************** FRAME PROCESSING ****************/
    void updatePlayerUI(QImage img);
    void processedPlayerUI(QImage processedImg);
    void dictionaryReceived(bool voc);
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
