#include "framefeatures.h"

FrameFeatures::FrameFeatures(QObject *parent) :
    QThread(parent)
{

}

void FrameFeatures::run()
{
    found = false;
    processFrames();
}

void FrameFeatures::processFrames()
{
    VideoCapture *capture  =  new cv::VideoCapture(filename);
    int numberOfFrames = capture->get(CV_CAP_PROP_FRAME_COUNT);
    const int selectedFrameNumber = 100;
    int Nth = numberOfFrames/selectedFrameNumber;

    const int hes_thresh = 1200;
    SurfFeatureDetector detector(hes_thresh);
    vector<KeyPoint> keypoints;
    int j = 0;
    Mat frm;
    capture->read(frm); // get a new frame from camera

    while(j < (numberOfFrames-Nth+1) && !frm.empty())
    {
        cvtColor(frm, frm, CV_BGR2GRAY);
        GaussianBlur(frm, frm, Size(7,7), 1.5, 1.5);
        detector.detect(frm, keypoints);
        featureVectorPerFrame.push_back(keypoints);

        qDebug() << "Size of bin = " << keypoints.size();

        j += Nth;
        capture->set(CV_CAP_PROP_POS_FRAMES, j);
        capture->read(frm);// get every Nth frame
        found = true;
    }

    emit onFeaturesFound(found);
}

void FrameFeatures::setFilename(string filename)
{
    this->filename = filename;
}

vector<vector<KeyPoint> > FrameFeatures::getFeatureVectors()
{
    return this->featureVectorPerFrame;
}
