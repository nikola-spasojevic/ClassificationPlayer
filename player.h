#ifndef PLAYER_H
#define PLAYER_H

#include <QMouseEvent>
#include <QMutex>
#include <QImage>
#include <QWaitCondition>
#include <QMainWindow>
#include <QFileDialog>
#include <QMessageBox>
#include <QTime>
#include "framefeatures.h"

class QLabel;

class Player : public QThread
{    Q_OBJECT
 private:
    bool stop;
    QMutex mutex;
    QWaitCondition condition;
    Mat frame;
    Mat processedFrame;
    int frameRate;
    VideoCapture *capture;
    Mat RGBframe;
    QImage img;
    QImage imgProcessed;
    unsigned int ID;

 signals:
    void originalImage(const QImage &image);
    void processedImage(const QImage &imgProcessed);

 public slots:
    void onFeaturesPassed(bool found);
    void onFFWpressed();
    void onFFWreleased();

 protected:
     void run();
     void msleep(int ms);

 public:
    Player(QObject *parent = 0);
    ~Player();
    bool loadVideo(string filename);
    void Play();
    void Stop();
    bool isStopped() const;
    void setCurrentFrame(int frameNumber);
    double getFrameRate();
    double getCurrentFrame();
    double getNumberOfFrames();
    void ProcessFrame();
    void getFeatureHeatMap();
    vector<vector<KeyPoint> > featureVectorPerFrame;
    FrameFeatures *frameFeatures;
};

#endif // PLAYER_H
