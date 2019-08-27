#include "frame.h"
#include <QPainter>
#include <QDebug>
#include <pcl/common/transforms.h>

std::string Frame::previous_visual_id;

Frame::Frame()
{

}

void Frame::Visualize(bool display_cropped) {
  if (trans) {
    Visualize(*trans, display_cropped);
  } else {
    Visualize(Eigen::Affine3f::Identity(), display_cropped);
  }
}

void Frame::Visualize(const Eigen::Affine3f& trans, bool display_cropped) {
//  trans.setIdentity();
//  trans.translate(Eigen::Vector3f{0, 0, 100});
//  Eigen::Matrix3f rotation;
//  rotation.setIdentity();
//  rotation = rotation * Eigen::AngleAxisf(90 * M_PI / 180.0f, Eigen::Vector3f::UnitX());
//  rotation = rotation * Eigen::AngleAxisf(90 * M_PI / 180.0f, Eigen::Vector3f::UnitZ());
//  trans.rotate(rotation);
//  trans.scale(2.0);

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
//      if ((qRed(rgb) + qGreen(rgb) + qBlue(rgb)) < 140) {
      if ((qRed(rgb) + qGreen(rgb) + qBlue(rgb)) < filter) {
        continue;
      }
      PointT point;
      point.x = col - half_w;
      point.y = row - half_h;
      point.z = 0.0f;
//      point.x *= scale;
//      point.y *= scale;
//      point.z *= scale;
      point.r = qRed(rgb);
      point.g = qGreen(rgb);
      point.b = qBlue(rgb);
      point.a = 255 * 0.5;

      Eigen::Vector3f transformed;
      pcl::transformPoint(Eigen::Vector3f(point.x, point.y, point.z), transformed, trans);
      point.x = transformed.x();
      point.y = transformed.y();
      point.z = transformed.z();
      cloud->push_back(point);
    }
  }

  if (label) {
    if (display_cropped) {
//      label->setPixmap(cropped.scaledToWidth(cropped.width() * 0.5));
      label->setPixmap(cropped.scaledToWidth(cropped.width() * 1.7));
    } else {
      label->setPixmap(image);
    }
  }

  if (viz) {
    viz->removePointCloud(previous_visual_id);
    viz->addPointCloud(cloud, "cloud" + std::to_string(id));
    previous_visual_id = "cloud" + std::to_string(id);
    viz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "cloud" + std::to_string(id));
    viz->setBackgroundColor(255, 255, 255);
  }
}

void Frame::Show() {
  QRect rect(kSplitLineColumn, 0, image.width(), image.height() - 40);
  QPixmap cropped = image.copy(rect);
//  label->setPixmap(cropped);
  QImage scan = cropped.toImage();
  PointCloudT::Ptr cloud(new PointCloudT);
  cloud->reserve(scan.width() * scan.height());
  int half_w = scan.width() / 2, half_h = scan.height() / 2;
  for (int row = 0; row < scan.height(); ++row) {
    for (int col = 0; col < scan.width(); ++col) {
      QRgb rgb = scan.pixel(col, row);
      if ((qRed(rgb) + qGreen(rgb) + qBlue(rgb)) < 140) { // 140, 10
        continue;
      }
      PointT point;
      point.x = col - half_w;
      point.y = row - half_h;
      point.z = 0.0f;
      point.r = qRed(rgb);
      point.g = qGreen(rgb);
      point.b = qBlue(rgb);
      point.a = 255 * 0.2;

      Eigen::Vector3f transformed;
      pcl::transformPoint(Eigen::Vector3f(point.x, point.y, point.z), transformed, trans ? *trans : Eigen::Affine3f::Identity());
      point.x = transformed.x();
      point.y = transformed.y();
      point.z = transformed.z();
      cloud->push_back(point);
    }
  }

  if (viz) {
    viz->removePointCloud("cut" + std::to_string(id));
    viz->addPointCloud(cloud, "cut" + std::to_string(id));
    viz->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "cut" + std::to_string(id));
    viz->setBackgroundColor(255, 255, 255);
  }
}

void Frame::Hide() {
  if (viz) {
    viz->removePointCloud("cut" + std::to_string(id));
  }
}

void Frame::RemoveVisual() {
  if (viz) {
    viz->removePointCloud("cloud" + std::to_string(id));
  }
}

std::string Frame::Write() {
  std::stringstream ss;
  auto data = trans->data();
  ss << id << " ";
  for (int i = 0; i < 16; ++i) {
    ss << data[i] << " ";
  }
  return ss.str();
}

std::vector<Frame> gFrames;
