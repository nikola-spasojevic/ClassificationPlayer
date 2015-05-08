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
    int Nth = frameRate;

    const int hes_thresh = 1500; //Set it as a Blob Detector
    SurfFeatureDetector detector(hes_thresh);
    detector.upright = 1; //orientation is not computed
    vector<KeyPoint> keypoints_scene;
    SurfDescriptorExtractor extractor;
    Mat descriptors_scene;

    int j = 0;
    Mat frm;
    capture->read(frm); // get a new frame from camera

    while(j < (numberOfFrames-Nth+1) && !frm.empty())
    {
        cvtColor(frm, frm, CV_BGR2GRAY);
        GaussianBlur(frm, frm, Size(7,7), 1.5, 1.5);

        //-- Step 1: Detect the keypoints using SURF Detector
        detector.detect(frm, keypoints_scene);

        //Choose top 50 Keypoints

        //-- Step 2: Calculate descriptors (feature vectors)
        extractor.compute(frm, keypoints_scene, descriptors_scene);

        descriptors_sceneVector.push_back(descriptors_scene);

        qDebug() << "Size of keypoints_scene bin = " << keypoints_scene.size();
        qDebug() << "Size of Scence Descriptor:  " << descriptors_sceneVector.size() << " " << descriptors_scene.rows << " x " << descriptors_scene.cols;

        j += Nth;
        capture->set(CV_CAP_PROP_POS_FRAMES, j);
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

void buildClassifier()
{
    /*
    CvANN_MLP mlp;
    CvTermCriteria criteria;
    criteria.max_iter = 100;
    criteria.epsilon = 0.00001f;
    criteria.type = CV_TERMCRIT_ITER | CV_TERMCRIT_EPS;
    CvANN_MLP_TrainParams params;
    params.train_method = CvANN_MLP_TrainParams::BACKPROP; params.bp_dw_scale = 0.05f;
    params.bp_moment_scale = 0.05f;
    params.term_crit = criteria;

    cv::Mat layers = cv::Mat(4, 1, CV_32SC1);

    layers.row(0) = cv::Scalar(2);
    layers.row(1) = cv::Scalar(10);
    layers.row(2) = cv::Scalar(15);
    layers.row(3) = cv::Scalar(1);

    mlp.create(layers);

    //mlp.train(trainingData , trainingClasses , cv::Mat(), cv::Mat(), params);

    */
}
