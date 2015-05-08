#include "mainwindow.h"
#include "ui_mainwindow.h"

const bool processed = false;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow)
{
    /**************** FRAME PROCESSING ****************/
    myPlayer = new Player();
    QObject::connect(myPlayer, SIGNAL(originalImage(QImage)), this, SLOT(updatePlayerUI(QImage)));
    QObject::connect(myPlayer, SIGNAL(processedImage(QImage)), this, SLOT(processedPlayerUI(QImage)));
    /**************** FRAME PROCESSING ****************/

    ui->setupUi(this);
    ui->PlyBtn->setEnabled(false);
    ui->ffwdBtn->setEnabled(false);
    ui->horizontalSlider->setEnabled(false);

    /**************** FEATURE SELECTION ****************/
    prevPoint = QPoint(0,0);
    topLeftCorner = QPoint(1600 ,1600);
    bottomRightCorner = QPoint(0,0);
    /**************** FEATURE SELECTION ****************/

    /**************** MOUSE TRACKING ****************/
    QWidget::connect(ui->outLabel, SIGNAL(Mouse_Move()), this, SLOT(Mouse_current_pos()));
    QWidget::connect(ui->outLabel, SIGNAL(Mouse_Pressed()), this, SLOT(Mouse_pressed()));
    QWidget::connect(ui->outLabel, SIGNAL(Mouse_Left()), this, SLOT(Mouse_left()));
    QWidget::connect(ui->outLabel, SIGNAL(Mouse_Release()), this, SLOT(Mouse_released()));
    /**************** MOUSE TRACKING ****************/
}

MainWindow::~MainWindow()
{
    delete ui;
}

void MainWindow::updatePlayerUI(QImage img)
{
    if(!processed)
    {
        if (!img.isNull())
        {
            px = QPixmap::fromImage(img);
            ui->outLabel->setScaledContents(true);
            ui->outLabel->setAlignment(Qt::AlignCenter);
            px = px.scaled(ui->outLabel->size(), Qt::KeepAspectRatio, Qt::FastTransformation);
            ui->outLabel->setPixmap(px);

            pxBuffer = px;
            ui->horizontalSlider->setValue(myPlayer->getCurrentFrame());
            ui->startTime->setText( getFormattedTime( (int)myPlayer->getCurrentFrame()/(int)myPlayer->getFrameRate()) );
        }
    }
}

void MainWindow::processedPlayerUI(QImage processedImg)
{
    if(processed)
    {
        if (!processedImg.isNull())
        {
            ui->outLabel->setScaledContents(true);
            ui->outLabel->setAlignment(Qt::AlignCenter);
            ui->outLabel->setPixmap(QPixmap::fromImage(processedImg).scaled(ui->outLabel->size(), Qt::KeepAspectRatio, Qt::FastTransformation));

            ui->horizontalSlider->setValue(myPlayer->getCurrentFrame());
            ui->startTime->setText( getFormattedTime( (int)myPlayer->getCurrentFrame()/(int)myPlayer->getFrameRate()) );
        }
    }
}


QString MainWindow::getFormattedTime(int timeInSeconds){

    int seconds = (int) (timeInSeconds) % 60 ;
    int minutes = (int) ((timeInSeconds / 60) % 60);
    int hours   = (int) ((timeInSeconds / (60*60)) % 24);

    QTime t(hours, minutes, seconds);
    if (hours == 0 )
        return t.toString("mm:ss");
    else
        return t.toString("h:mm:ss");
}

void MainWindow::on_PlyBtn_clicked()
{
    if (myPlayer->isStopped())
    {
        myPlayer->Play();
        ui->PlyBtn->setText(tr("Stop"));
    }else
    {
        myPlayer->Stop();
        ui->PlyBtn->setText(tr("Play"));
    }
}

void MainWindow::on_LdBtn_clicked()
{
    QString filename = QFileDialog::getOpenFileName(this, tr("Open Video"), ".", tr("Video Files (*.avi *.mpg *.mp4)"));
    QFileInfo name = filename;

    if (!filename.isEmpty())
    {
        if (!myPlayer->loadVideo(filename.toLatin1().data()))
        {
            QMessageBox msgBox;
            msgBox.setText("The selected video could not be opened!");
            msgBox.exec();
        }
        else
        {
           this->setWindowTitle(name.fileName());
           ui->PlyBtn->setEnabled(true);
           ui->ffwdBtn->setEnabled(true);
           ui->horizontalSlider->setEnabled(true);
           ui->horizontalSlider->setMaximum(myPlayer->getNumberOfFrames());
           ui->endTime->setText( getFormattedTime( (int)myPlayer->getNumberOfFrames()/(int)myPlayer->getFrameRate()) );
       }
    }
}

void MainWindow::on_horizontalSlider_sliderPressed()
{
    myPlayer->Stop();
}

void MainWindow::on_horizontalSlider_sliderReleased()
{
    myPlayer->Play();
}

void MainWindow::on_horizontalSlider_sliderMoved(int position)
{
    myPlayer->setCurrentFrame(position);
    ui->startTime->setText( getFormattedTime( position/(int)myPlayer->getFrameRate()) );
}

void MainWindow::on_ffwdBtn_pressed()
{ 
    const int FFValue = myPlayer->getFrameRate();
    int delay = (1000/FFValue);
    myPlayer->Stop();
    int j = myPlayer->getCurrentFrame();

    j += FFValue;
    myPlayer->setCurrentFrame(j);
    myPlayer->msleep(delay);
}

void MainWindow::on_ffwdBtn_released()
{
    myPlayer->Play();
}

void MainWindow::Mouse_current_pos()
{
    if (ui->outLabel->mouseHeld())
    {
        QPoint mouse_pos = ui->outLabel->mouseCurrentPos();

        if(mouse_pos.x() < ui->outLabel->width() && mouse_pos.y() < ui->outLabel->height() && mouse_pos.x() > 0 && mouse_pos.y() > 0 && prevPoint != QPoint(0,0))
        {   
            px = px.scaled(ui->outLabel->size());
            QPainter p(&px);
            QPen pen(Qt::red);
            pen.setWidth( 5 );
            p.setPen(pen);
            p.drawLine (mouse_pos.x(), mouse_pos.y(), prevPoint.x(), prevPoint.y());
            p.end ();
            ui->outLabel->setPixmap(px);

            if (mouse_pos.x() < topLeftCorner.x())
                topLeftCorner.setX(mouse_pos.x());
            if (mouse_pos.y() < topLeftCorner.y())
                topLeftCorner.setY(mouse_pos.y());
            if (mouse_pos.x() > bottomRightCorner.x())
                bottomRightCorner.setX(mouse_pos.x());
            if (mouse_pos.y() > bottomRightCorner.y())
                bottomRightCorner.setY(mouse_pos.y());
        }

        prevPoint = mouse_pos;
    }
}

void MainWindow::Mouse_pressed()
{
    myPlayer->Stop();
}

void MainWindow::Mouse_released()
{
    QPoint mouse_pos = ui->outLabel->mouseCurrentPos();
    pxBuffer = pxBuffer.scaled(ui->outLabel->size());

    if (myPlayer->isStopped() && mouse_pos.x() < ui->outLabel->width() && mouse_pos.y() < ui->outLabel->height() && mouse_pos.x() > 0 && mouse_pos.y() > 0)
    {
        roi = Rect(topLeftCorner.x(), topLeftCorner.y(), bottomRightCorner.x() - topLeftCorner.x(), bottomRightCorner.y() - topLeftCorner.y());

        cv::Mat frame = HelperFunctions::QPixmapToCvMat(pxBuffer);
        cv::Mat mask = frame(roi);

        processROI(mask);

        cv::namedWindow("Selected Feature");
        cv::imshow("Selected Feature", mask);

        myPlayer->Play();
        prevPoint = QPoint(0,0);
        topLeftCorner = QPoint(ui->outLabel->width() ,ui->outLabel->height());
        bottomRightCorner = QPoint(0,0);
    }
}

void MainWindow::Mouse_left()
{
    ui->outLabel->left = true;
}


void MainWindow::processROI(Mat roi)
{
    const int hes_thresh = 1500;
    SurfFeatureDetector detector(hes_thresh);
    detector.upright = 1; //orientation is not computed
    vector<KeyPoint> keypoints_object;
    SurfDescriptorExtractor extractor;
    Mat descriptors_object;

    cvtColor(roi, roi, CV_BGR2GRAY);
    GaussianBlur(roi, roi, Size(7,7), 1.5, 1.5);

    //-- Step 1: Detect the keypoints using SURF Detector
    detector.detect(roi, keypoints_object);

    //-- Step 2: Calculate descriptors (feature vectors)
    extractor.compute( roi, keypoints_object, descriptors_object);

    /*** Image Keypoint window***/
    //Mat img_keypoints;
    //cv::drawKeypoints(roi, keypoints_object, img_keypoints);
    //cv::imshow("Image Keypoints", img_keypoints);
    /*** Image Keypoint window***/

    //-- Step 3: Matching descriptor vectors using FLANN matcher
    FlannBasedMatcher matcher;
    std::vector< DMatch > matches;
    vector<cv::Mat> descriptors_sceneVector = myPlayer->frameFeatures->descriptors_sceneVector;

    for (unsigned int i = 0; i < descriptors_sceneVector.size(); i++)
    {
        cv::Mat descriptors_scene = descriptors_sceneVector.at(i);

        matcher.match( descriptors_object, descriptors_scene, matches );

        if (matches.size() > 4)
        {
            double max_dist = 0; double min_dist = 100;

            //-- Quick calculation of max and min distances between keypoints
            for( int i = 0; i < descriptors_object.rows; i++ )
            {
                double dist = matches[i].distance;
                if( dist < min_dist ) min_dist = dist;
                if( dist > max_dist ) max_dist = dist;
            }

            //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
            std::vector< DMatch > good_matches;

            for( int i = 0; i < descriptors_object.rows; i++ )
            {
                if( matches[i].distance < 3*min_dist )
                {
                    good_matches.push_back( matches[i]);
                }
            }

            Mat frame = myPlayer->

            Mat img_matches;
            drawMatches( roi, keypoints_object, frame, keypoints_frame,
                        good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                        vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );





        }








        //-- Localize the object
        std::vector<Point2f> obj;
        std::vector<Point2f> scene;

        for( int i = 0; i < good_matches.size(); i++ )
        {
            //-- Get the keypoints from the good matches
            obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
            scene.push_back( keypoints_scene[ good_matches[i].trainIdx ].pt );
        }

        Mat H = findHomography( obj, scene, CV_RANSAC );

        //-- Get the corners from the image_1 ( the object to be "detected" )
          std::vector<Point2f> obj_corners(4);
          obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( img_object.cols, 0 );
          obj_corners[2] = cvPoint( img_object.cols, img_object.rows ); obj_corners[3] = cvPoint( 0, img_object.rows );
          std::vector<Point2f> scene_corners(4);

          perspectiveTransform( obj_corners, scene_corners, H);

          //-- Draw lines between the corners (the mapped object in the scene - image_2 )
          //line( img_matches, scene_corners[0] + Point2f( img_object.cols, 0), scene_corners[1] + Point2f( img_object.cols, 0), Scalar(0, 255, 0), 4 );
          //line( img_matches, scene_corners[1] + Point2f( img_object.cols, 0), scene_corners[2] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
          //line( img_matches, scene_corners[2] + Point2f( img_object.cols, 0), scene_corners[3] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );
          //line( img_matches, scene_corners[3] + Point2f( img_object.cols, 0), scene_corners[0] + Point2f( img_object.cols, 0), Scalar( 0, 255, 0), 4 );

          //-- Show detected matches
          imshow( "Good Matches & Object detection", img_matches );
    }




}
