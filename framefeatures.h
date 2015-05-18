#ifndef FRAMEFEATURES_H
#define FRAMEFEATURES_H

#include <QThread>
#include <QtCore>
#include <QDebug>
#include <vector>

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/features2d/features2d.hpp>
#include <opencv2/calib3d/calib3d.hpp> // for homography
#include <opencv2/nonfree/nonfree.hpp>
#include <opencv2/video/video.hpp>
 #include <opencv2/objdetect/objdetect.hpp>
//#include <opencv2/ml/ml.hpp>
using namespace std;
using namespace cv;

class FrameFeatures : public QThread
{
    Q_OBJECT
public:
    explicit FrameFeatures(QObject *parent = 0);
    void run();
    void processFrames();
    void setFilename(string filename);
    vector<cv::Mat > getFeatureVectors();

    vector<cv::Mat> frameVector;
    vector<vector<KeyPoint> > keypoints_frameVector;
    vector<cv::Mat > descriptors_sceneVector;
    string filename;
    bool found;    

signals:
    void onFeaturesFound(const bool &found);

public slots:

};

#endif // FRAMEFEATURES_H
