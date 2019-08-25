#include "pclviewer.h"
#include "ui_pclviewer.h"
#include <QDir>
#include <QDebug>
#include <QTimer>
#include <frame.h>
#include "vein_tree.h"

const QString kMediaFolder = "media";
const QString kPlayButtonPlay = "Play";
const QString kPlayButtonStop = "Stop";
const QString kScanOnly = "Scan-Only Mode";
const QString kFullVideo = "Full-Video Mode";

PCLViewer::PCLViewer (QWidget *parent) :
  QMainWindow (parent),
  ui (new Ui::PCLViewer)
{
  ui->setupUi (this);
  this->setWindowTitle ("Liver6D");

  // Set up the QVTK window
  viewer.reset (new pcl::visualization::PCLVisualizer ("viewer", false));
  ui->qvtkWidget->SetRenderWindow (viewer->getRenderWindow ());
  viewer->setupInteractor (ui->qvtkWidget->GetInteractor (), ui->qvtkWidget->GetRenderWindow ());
  ui->qvtkWidget->update ();

  if (gFrames.empty()) {
    QDir directory(kMediaFolder);
    QStringList images = directory.entryList(QDir::Files);
    gFrames.reserve(images.size());
    for (auto& name : images) {
      Frame frame;
      frame.label = ui->ImageLabel;
      frame.viz = viewer;
      frame.id = gFrames.size();
      frame.image = QPixmap(kMediaFolder + "/" + name);
      gFrames.emplace_back(std::move(frame));
    }
  }

//  this->showFullScreen();

  ui->PlaySlider->setMinimum(0);
  ui->PlaySlider->setMaximum(gFrames.size() - 1);
  ui->PlaySlider->setValue(1);

  ui->scale->setMinimum(0.0);
  ui->scale->setValue(1.0);

  viewer->addCoordinateSystem(50);

  AddModelToViewer(viewer);
//  viewer->addPointCloud (cloud, "cloud");
//  viewer->resetCamera ();
//  ui->qvtkWidget->update ();
  viewer->resetCamera();

  timer_id_ = startTimer(ui->ScanIntervalEdit->text().toInt());

  connect(ui->px, SIGNAL(valueChanged(double)), this, SLOT(TransformEdited()));
  connect(ui->py, SIGNAL(valueChanged(double)), this, SLOT(TransformEdited()));
  connect(ui->pz, SIGNAL(valueChanged(double)), this, SLOT(TransformEdited()));
  connect(ui->ox, SIGNAL(valueChanged(double)), this, SLOT(TransformEdited()));
  connect(ui->oy, SIGNAL(valueChanged(double)), this, SLOT(TransformEdited()));
  connect(ui->oz, SIGNAL(valueChanged(double)), this, SLOT(TransformEdited()));
  connect(ui->scale, SIGNAL(valueChanged(double)), this, SLOT(TransformEdited()));
}

void PCLViewer::timerEvent(QTimerEvent *event) {
  if (event->timerId() != timer_id_) {
    return;
  }

  if (ui->PlayButton->text() == kPlayButtonStop) {
    auto slider = ui->PlaySlider;
    auto& frame = gFrames[ui->PlaySlider->value()];
    if (frame.trans) {
      frame.Visualize(ui->VideoModeButton->text() == kScanOnly);
      trans_ = *frame.trans;
    } else {
      frame.Visualize(trans_, ui->VideoModeButton->text() == kScanOnly);
    }
    // gFrames[slider->value()].Visualize(ui->VideoModeButton->text() == kScanOnly);
    slider->setValue(slider->value() + 1);
  }

  ui->qvtkWidget->update();
}

void PCLViewer::TransformEdited() {
  trans_.setIdentity();
  trans_.translate(Eigen::Vector3f(ui->px->value(), ui->py->value(), ui->pz->value()));
  trans_.rotate(
      Eigen::AngleAxisf(
          ui->ox->value() * M_PI / 180.0, Eigen::Vector3f::UnitX()) *
      Eigen::AngleAxisf(
          ui->oy->value() * M_PI / 180.0, Eigen::Vector3f::UnitY()) *
      Eigen::AngleAxisf(
          ui->oz->value() * M_PI / 180.0, Eigen::Vector3f::UnitZ()));
  trans_.scale(ui->scale->value());
  if (ui->PlayButton->text() == kPlayButtonPlay) {
    auto& frame = gFrames[ui->PlaySlider->value()];
    if (frame.trans) {
      frame.Visualize(ui->VideoModeButton->text() == kScanOnly);
    } else {
      frame.Visualize(trans_, ui->VideoModeButton->text() == kScanOnly);
    }
  }
}

PCLViewer::~PCLViewer ()
{
  delete ui;
}

void PCLViewer::on_PlayButton_clicked()
{
  auto button = ui->PlayButton;
  if (button->text() == kPlayButtonPlay) {
    button->setText(kPlayButtonStop);
  } else {
    button->setText(kPlayButtonPlay);
  }
}

void PCLViewer::on_PlaySlider_valueChanged(int value)
{
  auto slider = ui->PlaySlider;
  auto& frame = gFrames[ui->PlaySlider->value()];
  if (frame.trans) {
    frame.Visualize(ui->VideoModeButton->text() == kScanOnly);
    trans_ = *frame.trans;
  } else {
    frame.Visualize(trans_, ui->VideoModeButton->text() == kScanOnly);
  }
}

void PCLViewer::on_GoButton_clicked()
{
  auto slider = ui->PlaySlider;
  slider->setValue(slider->value() + 1);
  ui->PlayButton->setText(kPlayButtonPlay);
}

void PCLViewer::on_BackButton_clicked()
{
  auto slider = ui->PlaySlider;
  slider->setValue(slider->value() - 1);
  ui->PlayButton->setText(kPlayButtonPlay);
}

void PCLViewer::on_VideoModeButton_clicked()
{
  auto button = ui->VideoModeButton;
  if (button->text() == kScanOnly) {
    button->setText(kFullVideo);
  } else {
    button->setText(kScanOnly);
  }

  auto slider = ui->PlaySlider;
  auto& frame = gFrames[ui->PlaySlider->value()];
  if (frame.trans) {
    frame.Visualize(ui->VideoModeButton->text() == kScanOnly);
    trans_ = *frame.trans;
  } else {
    frame.Visualize(trans_, ui->VideoModeButton->text() == kScanOnly);
  }
}

void PCLViewer::on_ScanIntervalEdit_editingFinished()
{
  timer_id_ = startTimer(ui->ScanIntervalEdit->text().toInt());
}

void PCLViewer::AddModelToViewer(pcl::visualization::PCLVisualizer::Ptr viewer) {
  VeinTree* vein = new VeinTree();
  vein->Spawn(0, 0, -20);
  vein->viewer = viewer;
  vein->rgb = {0, 0, 1.0};

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

  vein->Plot();
//  pcl::ModelCoefficients coef;
//  coef.values = {0, 0, 0, 0, 0, 100, 10};
//  viewer->addCylinder(coef);
//  viewer->setShapeRenderingProperties( pcl::visualization::PCL_VISUALIZER_COLOR, 0.2, 0.45, 0, "cylinder");
}
