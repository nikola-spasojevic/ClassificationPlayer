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
    int frameRate = (int) capture->get(CV_CAP_PROP_FPS);
    int Nth = frameRate/2;
    int j = 0;
    Mat frm;
    capture->read(frm); // get a new frame from camera

    vector<KeyPoint> keypoints_scene;

    Mat descriptors_scene;

    Ptr<FeatureDetector> detector;
    detector =  new PyramidAdaptedFeatureDetector( new DynamicAdaptedFeatureDetector ( new FastAdjuster(40,true), 1000, 2000, 5), 4);
    Ptr<DescriptorExtractor> extractor = DescriptorExtractor::create("SURF");

    while( j < (numberOfFrames-Nth+1) && !frm.empty() )
    {
        keypoints_scene.clear();
        //cvtColor(frm, frm, CV_BGR2GRAY);

        //-- Step 1: Detect the keypoints using FAST Detector
        detector->detect(frm, keypoints_scene);

        //-- Step 2: Calculate SIFT descriptors (feature vectors)
        extractor->compute(frm, keypoints_scene, descriptors_scene);

        //-- Passing values to mainwindow.cpp
        frameVector.push_back(frm);
        keypoints_frameVector.push_back(keypoints_scene);
        descriptors_sceneVector.push_back(descriptors_scene);

        qDebug() << "Size of keypoints_scene bin = " << keypoints_scene.size();
        qDebug() << "Size of Scene Descriptor:  " << descriptors_sceneVector.size() << ": " << descriptors_scene.rows << " x " << descriptors_scene.cols;

        //j += Nth;
        //capture->set(CV_CAP_PROP_POS_FRAMES, j);
        capture->read(frm);// get every Nth frame (every second of video)
        found = true;
    }

    emit onFeaturesFound(found);
}

void FrameFeatures::setFilename(string filename)
{
    this->filename = filename;
}

vector<cv::Mat > FrameFeatures::getFeatureVectors()
{
    return this->descriptors_sceneVector;
}
