#include "mainwindow.h"
#include "ui_mainwindow.h"

const bool processed = false;

MainWindow::MainWindow(QWidget *parent) :
    QMainWindow(parent),
    ui(new Ui::MainWindow),
    bowDE(extractor, matcher)
{
    /**************** FRAME PROCESSING ****************/
    myPlayer = new Player();
    QObject::connect(myPlayer, SIGNAL(originalImage(QImage)), this, SLOT(updatePlayerUI(QImage)));
    QObject::connect(myPlayer, SIGNAL(processedImage(QImage)), this, SLOT(processedPlayerUI(QImage)));
    QObject::connect(myPlayer, SIGNAL(dictionaryPassed(bool)), this, SLOT(dictionaryReceived(bool)));
    /**************** FRAME PROCESSING ****************/

    ui->setupUi(this);
    ui->PlyBtn->setEnabled(false);
    ui->ffwdBtn->setEnabled(false);
    ui->horizontalSlider->setEnabled(false);

    /**************** FEATURE SELECTION ****************/
    window = cv::Rect(0, 0, 60, 60);//Window initialisation, setting width = 60, height = 60
    prevPoint = QPoint(0,0);
    topLeftCorner = QPoint(1600 ,1600);
    bottomRightCorner = QPoint(0,0);
    /**************** FEATURE SELECTION ****************/

    /**************** FEATURE DESCRIPTION AND EXTRACTION ****************/
    isDictionarySet = false;
    /**************** FEATURE DESCRIPTION AND EXTRACTION ****************/

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

void MainWindow::dictionaryReceived(bool voc)
{
    isDictionarySet = true;
    qDebug() << "is dictionary empty: " << myPlayer->dictionary.empty();
    bowDE.setVocabulary(myPlayer->dictionary);
    vector< vector<KeyPoint> >  featureVec = myPlayer->frameFeatures->keypoints_frameVector;

    for (int i = 0; i < featureVec.size(); i++)
    {
        cv::Mat frame = myPlayer->frameFeatures->frameVector.at(i);
        bowDE.compute(frame, featureVec.at(i), histogram_sceneVector.at(i));
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
            if (mouse_pos.x() > window.width)
                topLeftCorner.setX(mouse_pos.x() - window.width);
            else
                topLeftCorner.setX(0);

            if (mouse_pos.y() > window.height)
                topLeftCorner.setY(mouse_pos.y() - window.height);
            else
                topLeftCorner.setY(0);

            if (mouse_pos.x() < (ui->outLabel->width() - window.width))
                bottomRightCorner.setX(mouse_pos.x() + window.width);
            else
                bottomRightCorner.setX(ui->outLabel->width());

            if (mouse_pos.y() < (ui->outLabel->height() - window.height) )
                bottomRightCorner.setY(mouse_pos.y() + window.height);
            else
                bottomRightCorner.setY(ui->outLabel->height());
        }

        if (mouse_pos.x() < ui->outLabel->width() && mouse_pos.y() < ui->outLabel->height() && mouse_pos.x() > 0 && mouse_pos.y() > 0 && prevPoint != QPoint(0,0))
        {
            roi = Rect(topLeftCorner.x(), topLeftCorner.y(), bottomRightCorner.x() - topLeftCorner.x(), bottomRightCorner.y() - topLeftCorner.y());
            pxBuffer = pxBuffer.scaled(ui->outLabel->size());
            cv::Mat frame = HelperFunctions::QPixmapToCvMat(pxBuffer);
            cv::Mat mask = frame(roi);
            processROI(mask);
        }

        prevPoint = mouse_pos;
    }
}

void MainWindow::Mouse_pressed()
{
    prevPoint = QPoint(0,0);
}

void MainWindow::Mouse_released()
{

    /*
    //-- Step 3: Matching descriptor vectors using FLANN matcher
    //matcher = DescriptorMatcher::create("FlannBased");

      //Training the Bag of Words model with the selected feature components
    vector<Mat> descriptors = bowTrainer->getDescriptors();

    int count=0;
    for(vector<Mat>::iterator iter=descriptors.begin();iter!=descriptors.end();iter++)
    {
        count+=iter->rows;
    }
    qDebug() << "Clustering " << count << " features" << endl;

    if (count > MIN_FEATURE_SIZE && count > DICTIONARY_SIZE)
    {
        //choosing cluster's centroids as dictionary's words
        Mat dictionary = bowTrainer->cluster();

        BOWImgDescriptorExtractor bowDE(extractor, matcher);
        bowDE.setVocabulary(dictionary);
        qDebug() << "Processing training data..." << endl;
        qDebug() << "extracting histograms in the form of BOW for each image "<<endl;

        Mat trainingData(0, DICTIONARY_SIZE, CV_32FC1);
        Mat labels(0, 1, CV_32FC1);

        extractBOWDescriptor(trainingData, labels);

        NormalBayesClassifier classifier;
        qDebug() << "Training classifier..." << endl;

        classifier.train(trainingData, labels);

        qDebug() << "Processing evaluation data..." << endl;
        Mat evalData(0, DICTIONARY_SIZE, CV_32FC1);
        Mat groundTruth(0, 1, CV_32FC1);

    }
    */
}

void MainWindow::Mouse_left()
{
    ui->outLabel->left = true;
}

Mat MainWindow::connectedComponents(Mat roi)
{
   ///-- find dominant object via contours and calculate surf feature points within the bounded region--//
    cv::Mat roiBuffer = roi.clone();
    cv::Mat roiBufferCopy = roi.clone();
    cvtColor(roiBuffer, roiBuffer, CV_BGR2GRAY);

    // Threshold and morphology operations
    adaptiveThreshold(roiBuffer, roiBuffer, 255, ADAPTIVE_THRESH_GAUSSIAN_C,  THRESH_BINARY, 5, 0);
    cv::medianBlur(roiBuffer,roiBuffer,3);
    //equalizeHist(roiBuffer, roiBuffer);
    cv::erode(roiBuffer,roiBuffer,cv::Mat());
    cv::dilate(roiBuffer,roiBuffer,cv::Mat());
    GaussianBlur(roiBuffer, roiBuffer, Size(7,7), 1.5, 1.5);

    cv::vector<cv::vector<cv::Point> > contours;
    findContours( roiBuffer, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
    qDebug() << contours.size();

    /// Find contour with largest area
    double maxArea = 0;
    int maxIdx;
    for( int i = 0; i < contours.size(); i++ )
    {
        double ctrArea = contourArea(contours[i]);
        if ( maxArea < ctrArea)
        {
            maxArea = ctrArea;
            maxIdx = i;
        }
    }
    if (roiBuffer.size().area() == contourArea(contours[maxIdx]) )
        qDebug() << "Contour exceeds bounds!!!";
    qDebug() << maxArea;

    if (maxArea > COUNTOUR_AREA_THRESHOLD)
    {
        window.width *= 0.8;
        window.height*= 0.8;
    }
    else
    {
        window.width *= 1.2;
        window.height*= 1.2;
    }

    qDebug() << "window: " << window.width << " x " << window.height;

    Mat contourROI;
    Mat mask = Mat::zeros( roi.size(), roi.type());
    drawContours( mask, contours, maxIdx, Scalar(255,255,255), CV_FILLED, 8);

    drawContours( roiBufferCopy, contours, maxIdx, Scalar(0,150,0), 1.5 , 8);
    cv::Rect roi_temp(Point(topLeftCorner.x(), topLeftCorner.y()), roi.size());

    //Pixmap to Mat
    pxBuffer = pxBuffer.scaled(ui->outLabel->size());
    cv::Mat frame = HelperFunctions::QPixmapToCvMat(pxBuffer);
    roiBufferCopy.copyTo(frame(roi_temp));
    px = HelperFunctions::cvMatToQPixmap(frame);
    ui->outLabel->setScaledContents(true);
    ui->outLabel->setAlignment(Qt::AlignCenter);
    px = px.scaled(ui->outLabel->size(), Qt::KeepAspectRatio, Qt::FastTransformation);

    QPainter p(&px);
    QPen pen(Qt::red);
    pen.setWidth( 2 );
    p.setPen(pen);
    QPoint mouse_pos = ui->outLabel->mouseCurrentPos();
    p.drawLine (mouse_pos.x(), mouse_pos.y(), prevPoint.x(), prevPoint.y());
    p.end();

    ui->outLabel->setPixmap(px);

    bitwise_and(mask, roi, contourROI);
    return contourROI;
    ///-- find dominant object via contours and calculate surf feature points within the bounded region--//
}

void MainWindow::processROI(Mat roi)
{ 
    Mat contourROI = connectedComponents(roi);

    if (isDictionarySet)
    {
        vector<KeyPoint> keypoints_object;
        //Mat descriptors_object;
        Mat histogram;
        //std::vector<cv::Mat> img_matchesVector;

        //-- Step 1: Detect the keypoints
        detector->detect(contourROI, keypoints_object);

        //-- Step 2: Calculate descriptors (feature vectors)
        bowDE.compute(contourROI, keypoints_object, histogram);

        qDebug() << "histogram size: " << histogram.size().width << " x " << histogram.size().height;
    }
    /*
    if (!descriptors_object.empty())
    {
        bowTrainer->add(descriptors_object);
    }
    */
    /*

    vector<cv::Mat> descriptors_sceneVector = myPlayer->frameFeatures->descriptors_sceneVector;
    vector<vector<DMatch> > matches;

    for (unsigned int i = 0; i < descriptors_sceneVector.size(); i++)
    {
        cv::Mat descriptors_scene = descriptors_sceneVector.at(i);
        cv::Mat frame = myPlayer->frameFeatures->frameVector.at(i);
        vector<KeyPoint> keypoints_frame = myPlayer->frameFeatures->keypoints_frameVector.at(i);

        //Match the features found in the object roi with the features found in each scene using Fast Approximate Nearest Neighbor Search
        matcher->clear();
        matcher->knnMatch( descriptors_object, descriptors_scene, matches, 2);// Find two nearest matches

        //-- Draw only "good" matches (i.e. whose distance is less than 3*min_dist )
        std::vector< DMatch > good_matches;
        const float ratio = 0.82; // As in Lowe's paper; can be tuned

        for (int i = 0; i < matches.size(); ++i)
        {
            if (matches[i][0].distance < ratio * matches[i][1].distance)
            {
                good_matches.push_back(matches[i][0]);
            }
        }

        qDebug() << good_matches.size();

        //if there are more than 10 similar feature points then process and find similarities
        if (good_matches.size() >= 4)
        {
            Mat img_matches;
            drawMatches( contourROI, keypoints_object, frame, keypoints_frame,
                    good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
                    vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS );

            //-- Localize the object
            std::vector<Point2f> obj;
            std::vector<Point2f> scene;

            for( int i = 0; i < good_matches.size(); i++ )
            {
                //-- Get the keypoints from the good matches
                obj.push_back( keypoints_object[ good_matches[i].queryIdx ].pt );
                scene.push_back( keypoints_frame[ good_matches[i].trainIdx ].pt );
            }

            Mat H = findHomography( obj, scene, CV_RANSAC );

            if (HelperFunctions::niceHomography(&H))
            {
                //-- Get the corners from the object  to be "detected"
                std::vector<Point2f> obj_corners(4);
                obj_corners[0] = cvPoint(0,0); obj_corners[1] = cvPoint( roi.cols, 0 );
                obj_corners[2] = cvPoint( roi.cols, roi.rows ); obj_corners[3] = cvPoint( 0, roi.rows );
                std::vector<Point2f> scene_corners(4);

                perspectiveTransform( obj_corners, scene_corners, H);

                //-- Draw lines between the corners (the mapped object in the scene)
                line( img_matches, scene_corners[0] + Point2f( roi.cols, 0), scene_corners[1] + Point2f( roi.cols, 0), Scalar(0, 255, 0), 4 );
                line( img_matches, scene_corners[1] + Point2f( roi.cols, 0), scene_corners[2] + Point2f( roi.cols, 0), Scalar( 0, 255, 0), 4 );
                line( img_matches, scene_corners[2] + Point2f( roi.cols, 0), scene_corners[3] + Point2f( roi.cols, 0), Scalar( 0, 255, 0), 4 );
                line( img_matches, scene_corners[3] + Point2f( roi.cols, 0), scene_corners[0] + Point2f( roi.cols, 0), Scalar( 0, 255, 0), 4 );

                // Adjust the aspect ratio of the pre-processed frame
                cv::resize(img_matches, img_matches, cvSize(ui->outLabel->width()/2, ui->outLabel->height()/2));

                img_matchesVector.push_back(img_matches);
                Mat displayConcat = HelperFunctions::createOne(img_matchesVector, 1, 5);

                //-- Show detected matches
                startWindowThread();
                string s = "Good Matches & Object detection";
                imshow( s, displayConcat );
            }
        }
    }
*/
}


