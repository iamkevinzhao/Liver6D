#ifndef FRAME_H
#define FRAME_H

#include <QPixmap>
#include <vector>
#include <QLabel>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

class Frame
{
public:
  int id = 0;
  QPixmap image;
  QLabel* label = nullptr;
  const static int kSplitLineColumn = 270;
  // PointCloudT::Ptr cloud;
  pcl::visualization::PCLVisualizer::Ptr viz;

  Frame();
  void Visualize();
};

extern std::vector<Frame> gFrames;

#endif // FRAME_H
