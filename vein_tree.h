#ifndef VEIN_TREE_H
#define VEIN_TREE_H

#include <vector>
#include <tuple>
#include <pcl/visualization/pcl_visualizer.h>
#include <iostream>


class VeinTree
{
public:
  static int id;
  std::vector<VeinTree*> branches;
  float radius = 10;
  std::tuple<float, float, float> root;
  std::tuple<float, float, float> extreme;
  std::tuple<float, float, float> span;
  VeinTree() {
//    ++id;
    root = {0, 0, 0};
  }

  void Spawn(float dx, float dy, float dz) {
    span = {dx, dy, dz};
    float rx = std::get<0>(root);
    float ry = std::get<1>(root);
    float rz = std::get<2>(root);
    extreme = {rx + dx, ry + dy, rz + dz};
  }

  VeinTree(VeinTree* trunk) : VeinTree() {
    if (!trunk) {
      return;
    }
    root = trunk->extreme;
    radius = trunk->radius;
    viewer = trunk->viewer;
    rgb = trunk->rgb;
    trunk->branches.push_back(this);
  }

  pcl::visualization::PCLVisualizer::Ptr viewer;
  std::tuple<float, float, float> rgb;
  void Plot() {
    pcl::ModelCoefficients coef;
    coef.values =
        {std::get<0>(root), std::get<1>(root), std::get<2>(root),
         std::get<0>(span), std::get<1>(span), std::get<2>(span), radius};
    auto id = "vein" + std::to_string(this->id++);
    viewer->addCylinder(coef, id + "cyl");
    viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_COLOR,
          std::get<0>(rgb), std::get<1>(rgb), std::get<2>(rgb), id + "cyl");
    coef.values = {std::get<0>(extreme), std::get<1>(extreme), std::get<2>(extreme), radius};
    viewer->addSphere(coef, id + "sph");
    viewer->setShapeRenderingProperties(
          pcl::visualization::PCL_VISUALIZER_COLOR,
          std::get<0>(rgb), std::get<1>(rgb), std::get<2>(rgb), id + "sph");
    for (auto& branch : branches) {
      if (branch) {
        branch->Plot();
      }
    }
  }
};

#endif // VEIN_TREE_H
