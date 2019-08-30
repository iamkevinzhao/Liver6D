#include "pclviewer.h"
#include "vein_tree.h"

void PCLViewer::AddModelToViewer(pcl::visualization::PCLVisualizer::Ptr viewer) {
  VeinTree* vein = new VeinTree();
  vein->Spawn(0, 0, -20);
  vein->viewer = viewer;
  vein->rgb = {0, 0, 102.0 / 255};

  VeinTree* v;

  auto mhv = v = new VeinTree(vein);
  v->Spawn(0, 0, -15);
  v->radius *= 0.95f;

  v = new VeinTree(vein);
  v->Spawn(15, -2, -10);
  v->radius *= 0.7f;

  v = new VeinTree(v);
  v->Spawn(13, -2, -10);
  v->radius *= 0.95f;

  auto lhv1 = v = new VeinTree(v);
  v->Spawn(15, -1, -15);
  v->radius *= 0.95f;

  v = new VeinTree(v);
  v->Spawn(10, -1, -15);
  v->radius *= 0.95f;

  auto lhv2 = v = new VeinTree(v);
  v->Spawn(5, -1, -10);
  v->radius *= 0.95f;

  v = new VeinTree(v);
  v->Spawn(5, -1, -13);
  v->radius *= 0.95f;

  v = new VeinTree(v);
  v->Spawn(5, -1, -15);
  v->radius *= 0.95f;

  v = new VeinTree(v);
  v->Spawn(1, -1, -15);
  v->radius *= 0.95f;

  auto lhv3 = v = new VeinTree(v);
  v->Spawn(-3, -1, -15);
  v->radius *= 0.95f;

  v = new VeinTree(v);
  v->Spawn(-10, -1, -15);
  v->radius *= 0.95f;

  v = new VeinTree(lhv3);
  v->Spawn(10, -1, -15);
  v->radius *= 0.95f;

  v = new VeinTree(lhv2);
  v->Spawn(-5, -1, -10);
  v->radius *= 0.95f;

  v = new VeinTree(v);
  v->Spawn(-10, 1, -10);
  v->radius *= 0.95f;

  v = new VeinTree(lhv1);
  v->Spawn(20, 3, -5);
  v->radius *= 0.8f;

  v = new VeinTree(v);
  v->Spawn(20, 1, -2);
  v->radius *= 0.95f;

  auto mhv1 = v = new VeinTree(mhv);
  v->Spawn(-5, -10, -10);
  v->radius *= 0.7f;

  v = new VeinTree(v);
  v->Spawn(-15, -10, -7);
  v->radius *= 0.95f;

  v = new VeinTree(mhv1);
  v->Spawn(15, -5, -7);
  v->radius *= 0.95f;

  v = new VeinTree(v);
  v->Spawn(15, -5, -15);
  v->radius *= 0.95f;

  v = new VeinTree(v);
  v->Spawn(10, -3, -15);
  v->radius *= 0.95f;

  v = new VeinTree(v);
  v->Spawn(5, -3, -15);
  v->radius *= 0.95f;

  auto mhv2 = v = new VeinTree(v);
  v->Spawn(-0, -3, -15);
  v->radius *= 0.95f;

  v = new VeinTree(mhv2);
  v->Spawn(-5, -3, -15);
  v->radius *= 0.95f;

  v = new VeinTree(v);
  v->Spawn(-5, -1, -15);
  v->radius *= 0.95f;

  auto mhv3 = v = new VeinTree(v);
  v->Spawn(-10, -1, -15);
  v->radius *= 0.95f;

  v = new VeinTree(v);
  v->Spawn(-15, 1, -15);
  v->radius *= 0.95f;

  v = new VeinTree(v);
  v->Spawn(-20, 1, -15);
  v->radius *= 0.95f;

  v = new VeinTree(mhv3);
  v->Spawn(10, 2, -15);
  v->radius *= 0.9f;

  v = new VeinTree(mhv2);
  v->Spawn(10, -3, -11);
  v->radius *= 0.75f;

  v = new VeinTree(vein);
  v->Spawn(0, 0, -40);
  v->radius *= 0.95f;

  v = new VeinTree(v);
  v->Spawn(0, 0, -40);
  v->radius *= 0.95f;

//  v = new VeinTree(v);
//  v->Spawn(0, 0, -40);
//  v->radius *= 0.95f;

  v = new VeinTree(v);
  v->Spawn(0, 0, -40);
  v->radius *= 0.95f;

  v = new VeinTree(vein);
  v->Spawn(-20, 0, -5);
  v->radius *= 0.7f;

  v = new VeinTree(v);
  v->Spawn(-20, 0, -10);
  v->radius *= 0.95f;

  v = new VeinTree(v);
  v->Spawn(-15, 0, -15);
  v->radius *= 0.95f;

  auto rhv1 = v = new VeinTree(v);
  v->Spawn(-10, 0, -15);
  v->radius *= 0.95f;

  v = new VeinTree(rhv1);
  v->Spawn(10, -10, -20);
  v->radius *= 0.85f;

  v = new VeinTree(v);
  v->Spawn(15, -15, -20);
  v->radius *= 0.95f;

  v = new VeinTree(rhv1);
  v->Spawn(-15, 15, -10);
  v->radius *= 0.85f;

  v = new VeinTree(v);
  v->Spawn(-10, 10, -5);
  v->radius *= 0.95f;

  v = new VeinTree(rhv1);
  v->Spawn(-10, 0, -20);
  v->radius *= 0.95f;

  v = new VeinTree(v);
  v->Spawn(-5, 0, -20);
  v->radius *= 0.95f;

  v = new VeinTree(v);
  v->Spawn(-5, 0, -25);
  v->radius *= 0.95f;

  v = new VeinTree(v);
  v->Spawn(-5, 0, -15);
  v->radius *= 0.95f;

  auto rhv2 = v = new VeinTree(v);
  v->Spawn(-5, 0, -20);
  v->radius *= 0.95f;

  v = new VeinTree(rhv2);
  v->Spawn(-10, 10, -15);
  v->radius *= 0.95f;

  v = new VeinTree(v);
  v->Spawn(-5, 15, -15);
  v->radius *= 0.90f;

  v = new VeinTree(rhv2);
  v->Spawn(10, -10, -15);
  v->radius *= 0.90f;

  v = new VeinTree(v);
  v->Spawn(15, -15, -15);
  v->radius *= 0.95f;

  // vein->Plot();

  ////////////////////////////////////////////////

  VeinTree* portal = new VeinTree();
  portal->root = {-10, -10, -250};
  portal->Spawn(0, 0, 20);
  portal->viewer = viewer;
  portal->rgb = {51.0 / 255, 51.0 / 255, 1.0};

  v = new VeinTree(portal);
  v->Spawn(-2, 2, 20);
  v->radius *= 0.95f;

  auto pv1 = v = new VeinTree(v);
  v->Spawn(-1, 3, 20);
  v->radius *= 0.95f;

  v = new VeinTree(pv1);
  v->Spawn(6, 5, 10);
  v->radius *= 0.90f;

  auto lpv1 = v = new VeinTree(v);
  v->Spawn(12, 3, 7);
  v->radius *= 0.95f;

  v = new VeinTree(v);
  v->Spawn(6, 3, 14);
  v->radius *= 0.95f;

  v = new VeinTree(lpv1);
  v->Spawn(15, 0, 3);
  v->radius *= 0.90f;

  v = new VeinTree(v);
  v->Spawn(15, -1, 0);
  v->radius *= 0.95f;

  auto lpv2 = v = new VeinTree(v);
  v->Spawn(10, -3, -2);
  v->radius *= 0.95f;

  v = new VeinTree(lpv2);
  v->Spawn(6, -1, -3);
  v->radius *= 0.95;

  v = new VeinTree(v);
  v->Spawn(6, 1, -1);
  v->radius *= 1.0;

  v = new VeinTree(v);
  v->Spawn(6, 2, 1);
  v->radius *= 1.0;

  v = new VeinTree(lpv2);
  v->Spawn(3, -1, 7);
  v->radius *= 0.90;

  v = new VeinTree(v);
  v->Spawn(1, -2, 7);
  v->radius *= 0.95;

  v = new VeinTree(pv1);
  v->Spawn(-6, 5, 10);
  v->radius *= 0.90f;

  v = new VeinTree(v);
  v->Spawn(-10, 2, 5);
  v->radius *= 0.95f;

  auto rpv1 = v = new VeinTree(v);
  v->Spawn(-15, 2, 8);
  v->radius *= 0.95f;

  v = new VeinTree(rpv1);
  v->Spawn(-2, 2, 6);
  v->radius *= 0.95f;

  v = new VeinTree(v);
  v->Spawn(-1, 3, 10);
  v->radius *= 0.95f;

  v = new VeinTree(v);
  v->Spawn(0, 2, 10);
  v->radius *= 0.95f;

  v = new VeinTree(rpv1);
  v->Spawn(-15, 2, 2);
  v->radius *= 0.95f;

  v = new VeinTree(v);
  v->Spawn(-15, -1, -3);
  v->radius *= 0.95f;

  v = new VeinTree(v);
  v->Spawn(-12, -1, 0);
  v->radius *= 0.95f;

  // portal->Plot();

  //////////////////////////////////////

  VeinTree* ligament = new VeinTree();
  ligament->root = {40, -15, -195};
  ligament->Spawn(0, 5, 5);
  ligament->viewer = viewer;
  ligament->rgb = {102.0 / 255, 102.0 / 255, 1.0};
  ligament->radius *= 0.6f;

  v = new VeinTree(ligament);
  v->Spawn(1, 6, 4);
  v->radius *= 0.95f;

  v = new VeinTree(v);
  v->Spawn(2, 7, 3);
  v->radius *= 0.95f;

  v = new VeinTree(v);
  v->Spawn(3, 8, 2);
  v->radius *= 0.95f;

  v = new VeinTree(v);
  v->Spawn(1, 9, 3);
  v->radius *= 0.95f;


  // ligament->Plot();

  ///////////////////////////////////////

  Eigen::Affine3f trans;
  trans.setIdentity();
  trans.translate(Eigen::Vector3f(10, 20, -120));
  trans.rotate(
      Eigen::AngleAxisf(
          270 * M_PI / 180.0, Eigen::Vector3f::UnitX()) *
      Eigen::AngleAxisf(
          0 * M_PI / 180.0, Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(
          0 * M_PI / 180.0, Eigen::Vector3f::UnitZ()));
  trans.scale(1.1);

  QPixmap contour = QPixmap("liver_contour.jpg");
  QImage scan = contour.toImage();
  PointCloudT::Ptr cloud(new PointCloudT);
  cloud->reserve(scan.width() * scan.height());
  int half_w = scan.width() / 2, half_h = scan.height() / 2;
  for (int row = 0; row < scan.height(); ++row) {
    for (int col = 0; col < scan.width(); ++col) {
      QRgb rgb = scan.pixel(col, row);
//      if ((qRed(rgb) + qGreen(rgb) + qBlue(rgb)) < 140) {
      if ((qRed(rgb) + qGreen(rgb) + qBlue(rgb)) < 10) {
        continue;
      }
      if ((qRed(rgb) > 250) && (qGreen(rgb) > 250) && (qBlue(rgb) > 250)) {
        continue;
      }
      PointT point;
      point.x = col - half_w;
      point.y = row - half_h;
      point.z = 0.0f;
      point.r = qRed(rgb);
      point.g = qGreen(rgb);
      point.b = qBlue(rgb);
      point.a = 255 * 1.0;


      Eigen::Vector3f transformed;
      pcl::transformPoint(Eigen::Vector3f(point.x, point.y, point.z), transformed, trans);
      point.x = transformed.x();
      point.y = transformed.y();
      point.z = transformed.z();
      cloud->push_back(point);
    }
  }

  if (viewer) {
    viewer->addPointCloud(cloud, "contour");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "contour");
    viewer->setBackgroundColor(255, 255, 255);
  }
}
