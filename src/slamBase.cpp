#include"slamBase.h"

PointCloud::Ptr image2PointCloud(cv::Mat &rgb, cv::Mat &depth, CAMERA_INTRINSIC_PARAMETERS)
{
    PointCloud::Ptr cloud(new PointCloud);
    
    for(int m=0; m<depth.rows; m++){
        for(int n=0; n<dpeth.cols; n++){
            ushort d = depth.ptr<ushort>(m)[n];
            if(d<=0)
                continue;
            PointT p;
            p.z = double(d) / camera.scale;
            p.x = (n - camera.cx) * p.z / camera.fx;
            p.y = (m - camera.cy) * p.z / camera.fy;
            
            p.b = rgb.ptr<uchar>(m)[n*3];
            p.g = rgb.ptr<uchar>(m)[n*3+1];
            p.r = rgb.ptr<uchar>(m)[n*3+2];
            
            cloud->push_back(p);
                
       }
    }
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;
    return cloud;
}


Point3f point2dTo3d(cv::Point3f &point, CAMERA_INTRINSIC_PARAMETERS &camera)
{
    cv::Point3f p;
    p.z = double(point.z) / camera.scale;
    p.x = (point.x - camera.cx) * p.z / camera.fx;
    p.y = (point.y - camera.cy) * p.z / camera.fy;
    return p;
}


void computeKeyPointAndDesp(FRAME &frame, string detector, string descriptor)
{
    cv::Ptr<FeatureDetector> _detector;
    cv::Ptr<DescriptorExtractor> _descriptor;
    
    _detector = cv::FeatureDetector::create(detector.c_str());
    _descriptor = cv::DescriptorExtractor::create(descriptor.c_str());
    
    if(!_detector || !_descriptor){
        cerr<<"Unknown detector or descriptor type!"<<detector<<", "<<descriptor<<endl;
        return;
    }
    
    _detector->detect(frame.rgb, frame.kp);
    _descriptor->compute(frame.rgb, frame.kp, frame.desp);
    return;
}


RESULT_OF_PNP estimateMotion(FRAME &frame1, FRAME &frame2, CAMERA_INTRINSIC_PARAMETERS &camera)
{
    static ParameterReader pd;
    vector<cv::DMatch> matches;
    cv::BFMatcher matcher;
    matcher.match(frame1.desp, frame2.desp, matches);
    
    RESULT_OF_PNP result;
    vector<cv::DMatch> goodmatches;
    double minDis = 99999.0;
    double good_match_threshold = atof(pd.getData("good_match_threshold").s_str());
    
    for(size_t i=0; i<matches.size(); i++){
        if(matches[i].distance<minDis)
            minDis=matches[i].distance;
    }
    
    if(minDis<10)
        minDis=10;
    
    for(size_t i=0; i<matches.size(); i++){
        if(matches[i].distance<good_match_threshold*minDis)
            goodmatches.push_back(matches[i]);
    }
    
    if(goodmatches.size()<5){
        result.inliers = -1;
        return result;
    }
    
    vector<cv::Point3f> pts_obj;
    vector<cv::Point2f> pts_img;
    
    for(size_t i=0; i<goodmatches.size(); i++){
        cv::Point2f p = frame1.kp[goodmatches[i].queryIdx].pt;
        ushort d = frame1.depth.ptr<ushort>(int(p.y))[int(p.x)];
        if(d==0)
            continue;
        cv::Point3f pt(p.x, p.y, d);
        cv::Point3d pd = point2dTo3d(pt, camera);
        pts_obj.push_back(pd);
        
        pts_img.push_back(cv::Point2f(frame2.kp[goodmatches[i].trainIdx].pt));
    }
    
    if(pts_obj.size()==0 || pts_img.size()==0){
        result.inliers = -1;
        return result;
    }
    
    double camera_matrix_data[3][3] = {
        {camera.fx, 0, camera.cx},
        {0, camera.fy, camera.cy},
        {0,0,1}
    };
    
    cv::Mat cameraMatrix(3,3,CV_64F, camera_matrix_data); //构建相机矩阵
    cv::Mat rvec, tvec, inliers;
    
    cv::solvePnPRansac(pts_obj, pts_img, cameraMatrix, cv::Mat(), rvec, tvec, false, 100, 1.0, 100, inliers);
    
    result.rvec = rvec;
    result.tvec = tvec;
    result.inliers = inliers;
    
    return result;
}


Eigen::Isometry3d cvMat2Eigen(cv::Mat &rvec, cv::Mat &tvec)
{
    cv::Mat R;
    cv::Rodrigues(rvec, R);
    Eigen::Matrix3d r;
    for(int i=0; i<3; ++i)
        for(int j=0; i<3; ++j)
            r(i,j) = R.at<double>(i,j);
    
    Eigen::Isometry3d T = Eigen::Isometry3d::Identity();
    Eigen::AngleAxisd angle(r);
    T = angle;
    T(0,3) = tvec.at<double>(0,0);
    T(1,3) = tvec.at<double>(1,0);
    T(2,3) = tvec.at<double>(2,0);
    return T;
}




