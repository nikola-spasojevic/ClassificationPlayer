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
            p.end();
            ui->outLabel->setPixmap(px);

            if (mouse_pos.x() > 60)
                topLeftCorner.setX(mouse_pos.x() - 60);
            else
                topLeftCorner.setX(0);
            if (mouse_pos.y() > 60)
                topLeftCorner.setY(mouse_pos.y() - 60);
            else
                topLeftCorner.setY(0);
            if (mouse_pos.x() < (ui->outLabel->width() - 60))
                bottomRightCorner.setX(mouse_pos.x() + 60);
            else
                bottomRightCorner.setX(ui->outLabel->width());
            if (mouse_pos.y() < (ui->outLabel->height() - 60) )
                bottomRightCorner.setY(mouse_pos.y() + 60);
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
    QPoint mouse_pos = ui->outLabel->mouseCurrentPos();
    pxBuffer = pxBuffer.scaled(ui->outLabel->size());

    if (myPlayer->isStopped() && mouse_pos.x() < ui->outLabel->width() && mouse_pos.y() < ui->outLabel->height() && mouse_pos.x() > 0 && mouse_pos.y() > 0 && prevPoint != QPoint(0,0))
    {
        roi = Rect(topLeftCorner.x(), topLeftCorner.y(), bottomRightCorner.x() - topLeftCorner.x(), bottomRightCorner.y() - topLeftCorner.y());

        cv::Mat frame = HelperFunctions::QPixmapToCvMat(pxBuffer);
        cv::Mat mask = frame(roi);

        processROI(mask);

        //cv::namedWindow("Selected Feature");
        //cv::imshow("Selected Feature", mask);

        myPlayer->Play();
        prevPoint = QPoint(0,0);
        topLeftCorner = QPoint(ui->outLabel->width() ,ui->outLabel->height());
        bottomRightCorner = QPoint(0,0);
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
    adaptiveThreshold(roiBuffer, roiBuffer, 255, ADAPTIVE_THRESH_GAUSSIAN_C,  THRESH_BINARY, 3, 0);
    cv::medianBlur(roiBuffer,roiBuffer,3);
    cv::erode(roiBuffer,roiBuffer,cv::Mat());
    cv::dilate(roiBuffer,roiBuffer,cv::Mat());
    GaussianBlur(roiBuffer, roiBuffer, Size(7,7), 1.5, 1.5);

    cv::vector<cv::vector<cv::Point> > contours;
    findContours( roiBuffer, contours, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0) );
    qDebug() << contours.size();

    // Polygonal approximation
    vector<vector<Point> > contours_poly( contours.size() );
    vector<vector<Point> >hull( contours.size() );
    vector<Rect> boundRect( contours.size() );
    for( int i = 0; i < contours.size(); i++ )
    {
        approxPolyDP( Mat(contours[i]), contours_poly[i], 0.1*arcLength(contours[i],true), true );
        convexHull( Mat(contours[i]), hull[i], false );
        boundRect[i] = boundingRect( Mat(contours_poly[i]) );
    }

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
    qDebug() << maxArea;

    if (roiBuffer.size().area() == contourArea(contours[maxIdx]) )
        qDebug() << "Contour exceeds bounds!!!";

    Mat contourROI;
    Mat mask = Mat::zeros( roi.size(), roi.type());
    drawContours( mask, contours, maxIdx, Scalar(255,255,255), CV_FILLED, 8);

    drawContours( roiBufferCopy, contours, maxIdx, Scalar(0,150,0), 2, 8);
    cv::Rect roi_temp(Point(topLeftCorner.x(), topLeftCorner.y()), roi.size());

    //Pixmap to Mat
    pxBuffer = pxBuffer.scaled(ui->outLabel->size());
    cv::Mat frame = HelperFunctions::QPixmapToCvMat(pxBuffer);
    roiBufferCopy.copyTo(frame(roi_temp));
    px = HelperFunctions::cvMatToQPixmap(frame);
    ui->outLabel->setScaledContents(true);
    ui->outLabel->setAlignment(Qt::AlignCenter);
    px = px.scaled(ui->outLabel->size(), Qt::KeepAspectRatio, Qt::FastTransformation);
    ui->outLabel->setPixmap(px);
    bitwise_and(mask, roi, contourROI);
    return contourROI;
    ///-- find dominant object via contours and calculate surf feature points within the bounded region--//
}

void MainWindow::getCascade(Mat roi)
{
    equalizeHist(roi, roi);

    CascadeClassifier cascade;
    vector<Rect> objects;
    vector<int> reject_levels;
    vector<double> level_weights;
    const float scale_factor(1.2f);
    const int min_neighbors(3);
    cascade.detectMultiScale(roi, objects, reject_levels, level_weights, scale_factor, min_neighbors, CV_HAAR_FIND_BIGGEST_OBJECT);
    qDebug() << objects.size();
    Mat image_roi = roi.clone();
    for (int i = 0; i < objects.size(); i++)
    {
        rectangle( image_roi, objects[i], Scalar(255,0,0), 2, 8, 0);
        putText(image_roi, std::to_string(level_weights[i]), Point(objects[i].x, objects[i].y), 1, 1, Scalar(0,0,255));
        qDebug() << i;
    }
    if (objects.size() > 0)
        imshow("cascade", image_roi );
}

void MainWindow::processROI(Mat roi)
{ 
    HelperFunctions::cleanPreviousWindows();
    //cvtColor(roi, roi, CV_BGR2GRAY);
    Mat contourROI = connectedComponents(roi);
    Ptr<FeatureDetector> detector;
    detector = new PyramidAdaptedFeatureDetector(new DynamicAdaptedFeatureDetector ( new FastAdjuster(40,true), 500, 1000, 3), 4);
    Ptr<DescriptorExtractor> extractor = DescriptorExtractor::create("SURF");

    vector<KeyPoint> keypoints_object;
    Mat descriptors_object;
    std::vector<cv::Mat> img_matchesVector;

    //-- Step 1: Detect the keypoints
    detector->detect(contourROI, keypoints_object);
    if (keypoints_object.size() < 4)
        return;

    //-- Step 2: Calculate descriptors (feature vectors)
    extractor->compute( contourROI, keypoints_object, descriptors_object);

    //-- Step 3: Matching descriptor vectors using FLANN matcher
    vector<cv::Mat> descriptors_sceneVector = myPlayer->frameFeatures->descriptors_sceneVector;
    vector<vector<DMatch> > matches;
    Ptr<DescriptorMatcher> matcher = DescriptorMatcher::create("FlannBased");

    //bowTrainer->add(descriptors_object);

    /*
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


void MainWindow::clustering()
{
    /************* TRAINING VOCABULARY **************/
    //Training the Bag of Words model with the selected feature components

    vector<cv::Mat> descriptors_sceneVector = myPlayer->frameFeatures->descriptors_sceneVector;
    for (unsigned int i = 0; i < descriptors_sceneVector.size(); i++)
    {
        bowTrainer->add(descriptors_sceneVector.at(i));
    }
/*
        int count=0;
        for(vector<Mat>::iterator iter = descriptors_object.begin();iter!=descriptors_object.end();iter++)
        {
            count += iter->rows;
        }
        cout<<"Clustering "<<count<<" features"<<endl;

        //Mat dictionary = bowTrainer.cluster();

        //Create the Vocabulary with KMeans
        Mat vocabulary = bowTrainer->cluster();
        //qDebug() << vocabulary.size().height << " x " << vocabulary.size().width;

        //Since we now have a Vocabulary, compute the occurence of delegate in object
        BOWImgDescriptorExtractor bowDE( extractor, matcher);
        //Set the vocabulary
        bowDE.setVocabulary(vocabulary);
        cout<<"Processing training data..."<<endl;

        Mat trainingData(0, dictionarySize, CV_32FC1);
        Mat labels(0, 1, CV_32FC1);
        //extractBOWDescriptor(path(TRAINING_DATA_DIR), trainingData, labels);

        NormalBayesClassifier classifier;
            cout<<"Training classifier..."<<endl;

}*/
        /************* TRAINING VOCABULARY **************/
}

/*
void extractBOWDescriptor(const path& basepath, Mat& descriptors, Mat& labels)
{
    for (directory_iterator iter = directory_iterator(basepath); iter
            != directory_iterator(); iter++) {
        directory_entry entry = *iter;
        if (is_directory(entry.path())) {
            cout << "Processing directory " << entry.path().string() << endl;
            extractBOWDescriptor(entry.path(), descriptors, labels);
        } else {
            path entryPath = entry.path();
            if (entryPath.extension() == ".jpg") {
                cout << "Processing file " << entryPath.string() << endl;
                Mat img = imread(entryPath.string());
                if (!img.empty()) {
                    vector<KeyPoint> keypoints;
                    detector->detect(img, keypoints);
                    if (keypoints.empty()) {
                        cerr << "Warning: Could not find key points in image: "
                                << entryPath.string() << endl;
                    } else {
                        Mat bowDescriptor;
                        bowDE.compute(img, keypoints, bowDescriptor);
                        descriptors.push_back(bowDescriptor);
                        float label=atof(entryPath.filename().c_str());
                        labels.push_back(label);
                    }
                } else {
                    cerr << "Warning: Could not read image: "
                            << entryPath.string() << endl;
                }
            }
        }
    }
}
*/
