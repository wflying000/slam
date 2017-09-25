#include"slamBase.h"

PointCloud::Ptr image2PointCloud(cv::Mat &rgb, cv::Mat &depth, CAMERA_INTRINSIC_PARAMETERS)
{
    PointCloud::Ptr cloud(new PointCloud);
    
    for(int m=0; m<depth.rows; m++){
        for(int n=0; n<dpeth.cols; n++){
            ushort d = depth.ptr<ushort>(m)[n];
            if(d<=0)
                continue;
                
       }
    }
}
