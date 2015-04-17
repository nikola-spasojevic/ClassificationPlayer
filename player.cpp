#include "player.h"

Player::Player(QObject *parent)
 : QThread(parent)
{
    stop = true;
    frameFeatures = new FrameFeatures(this);
    QThread::connect(frameFeatures, SIGNAL(onFeaturesFound(bool)), this, SLOT(onFeaturesPassed(bool)));
}

Player::~Player()
{
    mutex.lock();
    stop = true;
    capture->release();
    delete capture;
    condition.wakeOne();
    mutex.unlock();
    wait();
}

bool Player::loadVideo(string filename) {
    capture  =  new cv::VideoCapture(filename);

    if (capture->isOpened())
    {
        frameRate = (int) capture->get(CV_CAP_PROP_FPS);
        ID = 0;
        frameFeatures->setFilename(filename);
        frameFeatures->start();
        return true;
    }
    else
        return false;
}

void Player::Play()
{
    if (!isRunning()) {
        if (isStopped()){
            stop = false;
        }
        start(LowPriority);
    }
}

void Player::Stop()
{
    stop = true;
}

void Player::msleep(int ms){
    struct timespec ts = { ms / 1000, (ms % 1000) * 1000 * 1000 };
    nanosleep(&ts, NULL);
}

bool Player::isStopped() const{
    return this->stop;
}

double Player::getCurrentFrame()
{
    return capture->get(CV_CAP_PROP_POS_FRAMES);
}

double Player::getNumberOfFrames()
{
    return capture->get(CV_CAP_PROP_FRAME_COUNT);
}

double Player::getFrameRate()
{
    return frameRate;
}

void Player::setCurrentFrame( int frameNumber )
{
    capture->set(CV_CAP_PROP_POS_FRAMES, frameNumber);
}

void Player::run()
{
    int delay = (1000/frameRate);
    while(!stop)
    {
        if (!capture->read(frame))
        {
            stop = true;
        }
        if (frame.channels()== 3)
        {
            cv::cvtColor(frame, RGBframe, CV_BGR2RGB);

            //ProcessFrame();
            img = QImage((const unsigned char*)(frame.data), frame.cols,frame.rows, QImage::Format_RGB888);
            imgProcessed = QImage((const unsigned char*)(processedFrame.data), processedFrame.cols,processedFrame.rows,QImage::Format_RGB888);

            emit originalImage(img);
            emit processedImage(imgProcessed);
        }
        else
        {
            cv::cvtColor(frame, RGBframe, CV_BGR2RGB);

            //ProcessFrame();
            img = QImage((const unsigned char*)(frame.data), frame.cols,frame.rows,QImage::Format_Indexed8);
            imgProcessed = QImage((const unsigned char*)(processedFrame.data), processedFrame.cols,processedFrame.rows,QImage::Format_Indexed8);

            emit originalImage(img);
            emit processedImage(imgProcessed);
        }

        this->msleep(delay);
    }
}

void Player::ProcessFrame()
{

}

void Player::onFeaturesPassed(bool found)
{
    printf("features have been found: %d \n", found);
    featureVectorPerFrame = frameFeatures->getFeatureVectors();
    //if (found)
        getFeatureHeatMap();
}

void Player::getFeatureHeatMap()
{
    vector<int> bins(featureVectorPerFrame.size()); //getNumberOfFrames();

    for(unsigned int i = 0; i < featureVectorPerFrame.size(); i++)
    {
        bins.at(i) = featureVectorPerFrame.at(i).size();
        qDebug() << "Size of bin " << i <<  "= " << bins.at(i);
    }

    cv::imshow("SURF Feature Heat Map", bins);
}

