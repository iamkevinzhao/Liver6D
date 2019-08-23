#include "frame.h"
#include <QPainter>
#include <QDebug>

Frame::Frame()
{

}

void Frame::Visualize() {
  if (label) {
    QPainter painter(&image);
    painter.setPen(Qt::red);
    label->setPixmap(image);
  }

  QRect rect(kSplitLineColumn, 0, image.width(), image.height() - 40);
  QPixmap cropped = image.copy(rect);
  label->setPixmap(cropped);
  QImage scan = cropped.toImage();
  PointCloudT::Ptr cloud(new PointCloudT);
  cloud->reserve(scan.width() * scan.height());
  int half_w = scan.width() / 2, half_h = scan.height() / 2;
  for (int row = 0; row < scan.height(); ++row) {
    for (int col = 0; col < scan.width(); ++col) {
      QRgb rgb = scan.pixel(col, row);
      if ((qRed(rgb) + qGreen(rgb) + qBlue(rgb)) < 140) {
        continue;
      }
      PointT point;
      point.x = col - half_w;
      point.y = row - half_h;
      point.z = 0.0f;
      point.r = qRed(rgb);
      point.g = qGreen(rgb);
      point.b = qBlue(rgb);
      point.a = 255;
      cloud->push_back(point);
    }
  }
  if (viz) {
    viz->removeAllPointClouds();
    viz->addPointCloud(cloud);
    viz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 2);
    viz->setBackgroundColor(255, 255, 255);
    // viz->spinOnce();
  }
}

std::vector<Frame> gFrames;
