/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include "feature_tracker_line.h"

FeatureTrackerLine::FeatureTrackerLine()
{
    stereo_cam = 0;
    n_id = 0;
    hasPrediction = false;
}
void FeatureTrackerLine::readIntrinsicParameter(const vector<string> &calib_file)
{
    for (size_t i = 0; i < calib_file.size(); i++)
    {
        ROS_INFO("reading paramerter of camera %s", calib_file[i].c_str());
        camodocal::CameraPtr camera = CameraFactory::instance()->generateCameraFromYamlFile(calib_file[i]);
        m_camera.push_back(camera);
    }
    if (calib_file.size() == 2)
        stereo_cam = 1;
}

vector<cv::Point2f> FeatureTrackerLine::undistortedPts(vector<cv::Point2f> &pts, camodocal::CameraPtr cam)
{
    vector<cv::Point2f> un_pts;
    for (unsigned int i = 0; i < pts.size(); i++)
    {
        Eigen::Vector2d a(pts[i].x, pts[i].y);
        Eigen::Vector3d b;
        cam->liftProjective(a, b);
        un_pts.push_back(cv::Point2f(b.x() / b.z(), b.y() / b.z()));
    }
    return un_pts;
}
map<int, vector<pair<int, Eigen::Vector4d>>> FeatureTrackerLine::trackImage(double _cur_time, const cv::Mat &_img, const cv::Mat &_img1)
{
    cv::Mat img;
    TicToc t_p;
    frame_cnt++;
    line_feature_id_use.clear();
    //cv::remap(_img, img, undist_map1_, undist_map2_, CV_INTER_LINEAR);
    img=_img;
    bool first_img = false;
    if (forwframe_ == nullptr) // 系统初始化的第一帧图像
    {
        forwframe_.reset(new FrameLines);
        curframe_.reset(new FrameLines);
        forwframe_->img =img;
        curframe_->img =img;
        first_img = true;
    }
    else
    {
        forwframe_.reset(new FrameLines);  // 初始化一个新的帧
        forwframe_->img =img;
    }

    TicToc t_li;

    Ptr<line_descriptor::LSDDetectorC> lsd_ = line_descriptor::LSDDetectorC::createLSDDetectorC();
    Ptr<line_descriptor::LSDDetectorC> lsd_p_right = line_descriptor::LSDDetectorC::createLSDDetectorC();
    // lsd parameters
    line_descriptor::LSDDetectorC::LSDOptions opts;
    opts.refine       = 1;     //1     	The way found lines will be refined
    opts.scale        = 0.5;   //0.8   	The scale of the image that will be used to find the lines. Range (0..1].
    opts.sigma_scale  = 0.6;	//0.6  	Sigma for Gaussian filter. It is computed as sigma = _sigma_scale/_scale.
    opts.quant        = 2.0;	//2.0   Bound to the quantization error on the gradient norm
    opts.ang_th       = 22.5;	//22.5	Gradient angle tolerance in degrees
    opts.log_eps      = 1.0;	//0		Detection threshold: -log10(NFA) > log_eps. Used only when advance refinement is chosen
    opts.density_th   = 0.6;	//0.7	Minimal density of aligned region points in the enclosing rectangle.
    opts.n_bins       = 1024;	//1024 	Number of bins in pseudo-ordering of gradient modulus.
    double min_line_length = 0.16;  // Line segments shorter than that are rejected
    opts.min_length   = min_line_length*(std::min(_img.cols,_img.rows));
    std::vector<KeyLine> lsd, keylsd,lsd_right,keylsd_right;
	//void LSDDetectorC::detect( const std::vector<Mat>& images, std::vector<std::vector<KeyLine> >& keylines, int scale, int numOctaves, const std::vector<Mat>& masks ) const
    lsd_->detect(img, lsd, 2, 1, opts);

    sum_time += t_li.toc();
    //ROS_INFO("line detect costs: %fms", t_li.toc());

    Mat lbd_descr, keylbd_descr,lbd_descr_right,keylbd_descr_right;
    // step 2: lbd descriptor
    TicToc t_lbd;
    Ptr<BinaryDescriptor> bd_ = BinaryDescriptor::createBinaryDescriptor();
    Ptr<BinaryDescriptor> bd_right = BinaryDescriptor::createBinaryDescriptor();
    bd_->compute(img, lsd, lbd_descr );


    for ( int i = 0; i < (int) lsd.size(); i++ )
    {
        if( lsd[i].octave == 0 && lsd[i].lineLength >= 60)
        {
            keylsd.push_back( lsd[i] );
            keylbd_descr.push_back( lbd_descr.row( i ) );
        }
    }
    
    
    sum_time += keylsd.size() * t_lbd.toc() / lsd.size();

    if(!_img1.empty() && stereo_cam)
    {
        lsd_p_right->detect(_img1, lsd_right, 2, 1, opts);
        sum_time += t_li.toc();
        bd_right->compute(_img1, lsd_right, lbd_descr_right);
        for ( int i = 0; i < (int) lsd_right.size(); i++ )
        {
            if( lsd_right[i].octave == 0 && lsd_right[i].lineLength >= 60)
            {
                keylsd_right.push_back( lsd_right[i] );
                keylbd_descr_right.push_back( lbd_descr_right.row( i ) );
            }
        }
    }
    forwframe_->keylsd = keylsd;
    forwframe_->lbd_descr = keylbd_descr;
    for (size_t i = 0; i < forwframe_->keylsd.size(); ++i) {
        if(first_img)
            forwframe_->lineID.push_back(allfeature_cnt++);
        else
            forwframe_->lineID.push_back(-1);   // give a negative id
    }

    if(curframe_->keylsd.size() > 0)
    {
        /* compute matches */
        TicToc t_match;
        std::vector<DMatch> lsd_matches;
        Ptr<BinaryDescriptorMatcher> bdm_;
        bdm_ = BinaryDescriptorMatcher::createBinaryDescriptorMatcher();
        bdm_->match(forwframe_->lbd_descr, curframe_->lbd_descr, lsd_matches);
        //ROS_INFO("lbd_macht costs: %fms", t_match.toc()); 
        sum_time += t_match.toc();
        mean_time = sum_time/frame_cnt;
        // ROS_INFO("line feature tracker mean costs: %fms", mean_time);
        /* select best matches */
        std::vector<DMatch> good_matches;
        std::vector<KeyLine> good_Keylines;
        good_matches.clear();
        for ( int i = 0; i < (int) lsd_matches.size(); i++ )
        {
            if( lsd_matches[i].distance < 30 ){

                DMatch mt = lsd_matches[i];
                KeyLine line1 =  forwframe_->keylsd[mt.queryIdx] ;
                KeyLine line2 =  curframe_->keylsd[mt.trainIdx] ;
                Point2f serr = line1.getStartPoint() - line2.getStartPoint();
                Point2f eerr = line1.getEndPoint() - line2.getEndPoint();
                // std::cout<<"11111111111111111 = "<<abs(line1.angle-line2.angle)<<std::endl;
                if((serr.dot(serr) < 80 * 80) && (eerr.dot(eerr) < 80 * 80)&&abs(line1.angle-line2.angle)<0.1&&line_feature_id_use[curframe_->lineID[mt.trainIdx]]==0)   // 线段在图像里不会跑得特别远
                {
                    good_matches.push_back( lsd_matches[i] );
                    line_feature_id_use[curframe_->lineID[mt.trainIdx]]=1;
                }
            }
        }

        vector< int > success_id;
        // std::cout << forwframe_->lineID.size() <<" " <<curframe_->lineID.size();
        for (int k = 0; k < good_matches.size(); ++k) {
            DMatch mt = good_matches[k];
            forwframe_->lineID[mt.queryIdx] = curframe_->lineID[mt.trainIdx];
            success_id.push_back(curframe_->lineID[mt.trainIdx]);
        }

        //把没追踪到的线存起来
        vector<KeyLine> vecLine_tracked, vecLine_new;
        vector< int > lineID_tracked, lineID_new;
        Mat DEscr_tracked, Descr_new;
        // 将跟踪的线和没跟踪上的线进行区分
        for (size_t i = 0; i < forwframe_->keylsd.size(); ++i)
        {
            if( forwframe_->lineID[i] == -1)
            {
                forwframe_->lineID[i] = allfeature_cnt++;
                vecLine_new.push_back(forwframe_->keylsd[i]);
                lineID_new.push_back(forwframe_->lineID[i]);
                Descr_new.push_back( forwframe_->lbd_descr.row( i ) );
            }
            
            else
            {
                vecLine_tracked.push_back(forwframe_->keylsd[i]);
                lineID_tracked.push_back(forwframe_->lineID[i]);
                DEscr_tracked.push_back( forwframe_->lbd_descr.row( i ) );
            }
        }

        vector<KeyLine> h_Line_new, v_Line_new;
        vector< int > h_lineID_new,v_lineID_new;
        Mat h_Descr_new,v_Descr_new;
        for (size_t i = 0; i < vecLine_new.size(); ++i)
        {
            if((((vecLine_new[i].angle >= 3.14/4 && vecLine_new[i].angle <= 3*3.14/4))||(vecLine_new[i].angle <= -3.14/4 && vecLine_new[i].angle >= -3*3.14/4)))
            {
                h_Line_new.push_back(vecLine_new[i]);
                h_lineID_new.push_back(lineID_new[i]);
                h_Descr_new.push_back(Descr_new.row( i ));
            }
            else
            {
                v_Line_new.push_back(vecLine_new[i]);
                v_lineID_new.push_back(lineID_new[i]);
                v_Descr_new.push_back(Descr_new.row( i ));
            }      
        }
        int h_line,v_line;
        h_line = v_line =0;
        for (size_t i = 0; i < vecLine_tracked.size(); ++i)
        {
            if((((vecLine_tracked[i].angle >= 3.14/4 && vecLine_tracked[i].angle <= 3*3.14/4))||(vecLine_tracked[i].angle <= -3.14/4 && vecLine_tracked[i].angle >= -3*3.14/4)))
            {
                h_line ++;
            }
            else
            {
                v_line ++;
            }
        }
        int diff_h = 25 - h_line;
        int diff_v = 25 - v_line;
        // std::cout<<"h_line = "<<h_line<<" v_line = "<<v_line<<std::endl;
        if( diff_h > 0)    // 补充线条
        {
            int kkk = 1;
            if(diff_h > h_Line_new.size())
                diff_h = h_Line_new.size();
            else 
                kkk = int(h_Line_new.size()/diff_h);
            for (int k = 0; k < diff_h; ++k) 
            {
                vecLine_tracked.push_back(h_Line_new[k]);
                lineID_tracked.push_back(h_lineID_new[k]);
                DEscr_tracked.push_back(h_Descr_new.row(k));
            }
            // std::cout  <<"h_kkk = " <<kkk<<" diff_h = "<<diff_h<<" h_Line_new.size() = "<<h_Line_new.size()<<std::endl;
        }
        if( diff_v > 0)    // 补充线条
        {
            int kkk = 1;
            if(diff_v > v_Line_new.size())
                diff_v = v_Line_new.size();
            else 
                kkk = int(v_Line_new.size()/diff_v);
            for (int k = 0; k < diff_v; ++k)  
            {
                vecLine_tracked.push_back(v_Line_new[k]);
                lineID_tracked.push_back(v_lineID_new[k]);
                DEscr_tracked.push_back(v_Descr_new.row(k));
            }   // std::cout  <<"v_kkk = " <<kkk<<" diff_v = "<<diff_v<<" v_Line_new.size() = "<<v_Line_new.size()<<std::endl;
        }
        forwframe_->keylsd = vecLine_tracked;
        forwframe_->lineID = lineID_tracked;
        forwframe_->lbd_descr = DEscr_tracked;
    }
    
    map<int, vector<pair<int, Eigen::Vector4d>>> featureFrame;
    //printf("this frame line feature id ");
    for (int j = 0; j < forwframe_->keylsd.size(); ++j) {
        Line l;
        KeyLine lsd = forwframe_->keylsd[j];
        l.StartPt = lsd.getStartPoint();
        l.EndPt = lsd.getEndPoint();
        l.length = lsd.lineLength;
        forwframe_->vecLine.push_back(l);
        int feature_id=forwframe_->lineID[j];
        int camera_id=0;
        vector<cv::Point2f>a(2);
        a[0]=l.StartPt;
        a[1]=l.EndPt;
        //printf("%d ",feature_id);
        vector<cv::Point2f>b=undistortedPts(a,m_camera[0]);
        featureFrame[feature_id].emplace_back(camera_id,  Eigen::Vector4d(b[0].x,b[0].y,b[1].x,b[1].y));
    }
    line_feature_id_use.clear();
    if(!_img1.empty() && stereo_cam)
    {
        TicToc t_match;
        std::vector<DMatch> lsd_matches;
        Ptr<BinaryDescriptorMatcher> bdm_;
        bdm_ = BinaryDescriptorMatcher::createBinaryDescriptorMatcher();
        bdm_->match(forwframe_->lbd_descr, lbd_descr_right, lsd_matches);
        //ROS_INFO("lbd_macht costs: %fms", t_match.toc()); 
        sum_time += t_match.toc();
        mean_time = sum_time/frame_cnt;
        // ROS_INFO("line feature tracker mean costs: %fms", mean_time);
        /* select best matches */
        std::vector<DMatch> good_matches;
        //std::vector<KeyLine> good_Keylines;
        good_matches.clear();
        for ( int i = 0; i < (int) lsd_matches.size(); i++ )
        {
            if( lsd_matches[i].distance < 30 ){
                DMatch mt = lsd_matches[i];
                KeyLine line1 =  forwframe_->keylsd[mt.queryIdx] ;
                KeyLine line2 =  keylsd_right[mt.trainIdx] ;
                Point2f serr = line1.getStartPoint() - line2.getStartPoint();
                Point2f eerr = line1.getEndPoint() - line2.getEndPoint();
                // std::cout<<"11111111111111111 = "<<abs(line1.angle-line2.angle)<<std::endl;
                if((serr.dot(serr) < 80 * 80) && (eerr.dot(eerr) < 80 * 80)&&abs(line1.angle-line2.angle)<0.1&&line_feature_id_use[forwframe_->lineID[mt.queryIdx]]==0)   // 线段在图像里不会跑得特别远
                {
                    Line l;
                    l.StartPt = line2.getStartPoint();
                    l.EndPt = line2.getEndPoint();
                    l.length = line2.lineLength;
                    int feature_id=forwframe_->lineID[mt.queryIdx];
                    int camera_id=1;
                    vector<cv::Point2f>a(2);
                    a[0]=l.StartPt;
                    a[1]=l.EndPt;
                    //printf("%d ",feature_id);
                    vector<cv::Point2f>b=undistortedPts(a,m_camera[1]);
                    line_feature_id_use[forwframe_->lineID[mt.queryIdx]]=1;
                    featureFrame[feature_id].emplace_back(camera_id,  Eigen::Vector4d(b[0].x,b[0].y,b[1].x,b[1].y));
                }
            }
        }
    }
    //printf("\n");
    curframe_ = forwframe_;
    if(SHOW_TRACK)
        drawTrack(img, img);
    for(auto id : featureFrame)printf("%d ",id.second.size());printf("\n");
    return featureFrame;
}
void FeatureTrackerLine::drawTrack(const cv::Mat &imLeft, const cv::Mat &imRight)
{
    int cols = imLeft.cols;
    // if (!imRight.empty() && stereo_cam)
    //     cv::hconcat(imLeft, imRight, imTrack);
    // else
    imTrack = imLeft.clone();
    cv::cvtColor(imTrack, imTrack, CV_GRAY2RGB);
    for (size_t j = 0; j < curframe_->vecLine.size(); j++)
    {
        double x0=curframe_->vecLine[j].StartPt.x,y0=curframe_->vecLine[j].StartPt.y;
        double x1=curframe_->vecLine[j].EndPt.x,y1=curframe_->vecLine[j].EndPt.y;
        cv::Point start(x0, y0);
        cv::Point end(x1, y1);
        cv::line(imTrack, start, end, cv::Scalar(0, 0, 255), 2);
    }
    for (size_t j = 0; j < curframe_->keylsd.size(); j++)
    {
        double x0=curframe_->keylsd[j].getStartPoint().x,y0=curframe_->keylsd[j].getStartPoint().y;
        double x1=curframe_->keylsd[j].getEndPoint().x,y1=curframe_->keylsd[j].getEndPoint().y;
        cv::Point start(x0, y0);
        cv::Point end(x1, y1);
        cv::line(imTrack, start, end, cv::Scalar(255, 0, 0), 1);
    }
}
cv::Mat FeatureTrackerLine::getTrackImage()
{
    return imTrack;
}



