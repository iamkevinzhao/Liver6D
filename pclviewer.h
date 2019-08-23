#pragma once

#include <iostream>

// Qt
#include <QMainWindow>

// Point Cloud Library
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

// Visualization Toolkit (VTK)
#include <vtkRenderWindow.h>

typedef pcl::PointXYZRGBA PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

namespace Ui
{
  class PCLViewer;
}

class PCLViewer : public QMainWindow
{
  Q_OBJECT

public:
  explicit PCLViewer (QWidget *parent = 0);
  ~PCLViewer ();

protected:
  void timerEvent(QTimerEvent* event);

  pcl::visualization::PCLVisualizer::Ptr viewer;

private slots:
  void on_PlayButton_clicked();

  void on_PlaySlider_valueChanged(int value);

  void on_GoButton_clicked();

  void on_BackButton_clicked();

private:
  Ui::PCLViewer *ui;
  int timer_id_;
};