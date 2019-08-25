#include "frame.h"
#include <QPainter>
#include <QDebug>

Frame::Frame()
{

}

void Frame::Visualize(bool display_cropped) {
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
      point.x *= scale;
      point.y *= scale;
      point.z *= scale;
      point.r = qRed(rgb);
      point.g = qGreen(rgb);
      point.b = qBlue(rgb);
      point.a = 255;
      cloud->push_back(point);
    }
  }

  if (label) {
    if (display_cropped) {
      label->setPixmap(cropped.scaledToWidth(cropped.width() * 1.7));
    } else {
      label->setPixmap(image);
    }
  }

  if (viz) {
    viz->removeAllPointClouds();
    viz->addPointCloud(cloud);
    viz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3);
    viz->setBackgroundColor(255, 255, 255);
  }
}

std::vector<Frame> gFrames;
