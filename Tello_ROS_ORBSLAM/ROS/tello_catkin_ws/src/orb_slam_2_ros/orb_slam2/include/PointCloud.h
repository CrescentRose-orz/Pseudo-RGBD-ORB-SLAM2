#ifndef POINTCLOUD_H
#define POINTCLOUD_H
/** * * ━━━━━━神兽出没━━━━━━ 
* 　　　┏┓　　　┏┓ 
* 　　┃　　　　　　　┃ 
* 　　┃　　　━　　　┃ 
* 　　┃　┳┛　┗┳　┃ 
* 　　┃　　　　　　　┃ 
* 　　┃　　　┻　　　┃ 
* 　　┃　　　　　　　┃ 
* 　　┗━┓　　　┏━┛Code is far away from bug with the animal rotecting 
* 　　　　┃　　　┃ 神兽保佑,代码无bug 
* 　　　　┃　　　┃ 
* 　　　　┃　　　┗━━━┓ 
* 　　　　┃　　　　　　　┣┓ 
* 　　　　┃　　　　　　　┏┛ 
* 　　　　┗┓┓┏━┳┓┏┛ 
* 　　　　　┃┫┫　┃┫┫ 
* 　　　　　┗┻┛　┗┻┛ 
* * ━━━━━━感觉萌萌哒━━━━━━ */ /** 
* 　　　　　　　　┏┓　　　┏┓ 
* 　　　　　　　┏┛┻━━━┛┻┓ 
* 　　　　　　　┃　　　　　　 ┃ 　 
* 　　　　　　　┃　　　━　　 ┃ 
* 　　　　　　　┃　＞ ＜  ┃ 
* 　　　　　　　┃　　　　　　 ┃ 
* 　　　　　　　┃...　⌒　.┃ 
* 　　　　　　　┃　　　　　　　┃ 
* 　　　　　　　┗━┓　　　┏━┛ 
* 　　　　　　　　　┃　　　┃　Code is far away from bug with the animal protecting　　　　　　　　　　 
* 　　　　　　　　　┃　　　┃ 神兽保佑,代码无bug 
* 　　　　　　　　　┃　　　┃　　　　　　　　　　　 
* 　　　　　　　　　┃　　　┃ 　　　　　　 
* 　　　　　　　　　┃　　　┃ 
* 　　　　　　　　　┃　　　┃　　　　　　　　　　　 
* 　　　　　　　　　┃　　　┗━━━┓ 
* 　　　　　　　　　┃　　　　　　　┣┓ 
* 　　　　　　　　　┃　　　　　　　┏┛ 
* 　　　　　　　　　┗┓┓┏━┳┓┏┛ 
* 　　　　　　　　　　┃┫┫　┃┫┫ 
* 　　　　　　　　　　┗┻┛　┗┻┛ 
*
/ /** 

*　　　　　　　　┏┓　　　┏┓+ + 
*　　　　　　　┏┛┻━━━┛┻┓ + + 
*　　　　　　　┃　　　　　　　┃ 　 
*　　　　　　　┃　　　━　　　┃ ++ + + + 
*　　　　　　 ████━████ ┃+ 
*　　　　　　　┃　　　　　　　┃ + 
*　　　　　　　┃　　　┻　　　┃ 
*　　　　　　　┃　　　　　　　┃ + + 
*　　　　　　　┗━┓　　　┏━┛ 
*　　　　　　　　　┃　　　┃　　　　　　　　　　　 
*　　　　　　　　　┃　　　┃ + + + + 
*　　　　　　　　　┃　　　┃　　　　Code is far away from bug with the animal protecting　　　　　　　 
*　　　　　　　　　┃　　　┃ + 　　　　神兽保佑,代码无bug　　 
*　　　　　　　　　┃　　　┃ 
*　　　　　　　　　┃　　　┃　　+　　　　　　　　　 
*　　　　　　　　　┃　 　　┗━━━┓ + + 
*　　　　　　　　　┃ 　　　　　　　┣┓ 
*　　　　　　　　　┃ 　　　　　　　┏┛ 
*　　　　　　　　　┗┓┓┏━┳┓┏┛ + + + + 
*　　　　　　　　　　┃┫┫　┃┫┫ 
*　　　　　　　　　　┗┻┛　┗┻┛+ + + + 
*/
//#include "PointcloudMapping.h"
//#include "Frame.h"
//#include "Map.h"
#include <pcl/common/transforms.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include <condition_variable>
#include <pcl/io/pcd_io.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <opencv2/core/core.hpp>
#include <mutex>

namespace ORB_SLAM2
{

class myPointCloud
{
    typedef pcl::PointXYZRGB PointT;
    typedef pcl::PointCloud<PointT> mcPointCloud;
public:
    mcPointCloud::Ptr pcE;
public:
    Eigen::Isometry3d T;
    myPointCloud();
    int pcID; 
//protected:    
public:
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
  // DONT DELETE !!!!!!!!!!!!　coredump here // eigen3 有sse128 加速，故需要特殊对齐,当使用ｓｔｌ容器或类成员时加此条   
};

} //namespace ORB_SLAM

#endif // POINTCLOUDE_H