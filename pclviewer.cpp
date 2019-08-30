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

bool gProtect = false;

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
//      if (frame.id % 100 == 0) {
//        frame.trans = new Eigen::Affine3f;
//        frame.trans->setIdentity();
//        frame.trans->rotate(Eigen::AngleAxisf(frame.id, Eigen::Vector3f::UnitX()));
//        frame.Show();
//      }
      gFrames.emplace_back(std::move(frame));
    }
  }

  SetFramesFromFile();

  this->showFullScreen();

  ui->PlaySlider->setMinimum(0);
  ui->PlaySlider->setMaximum(gFrames.size() - 1);
  ui->PlaySlider->setValue(1);

  ui->scale->setMinimum(0.0);
  ui->scale->setValue(1.0);
  ui->alpha->setValue(1.0);
  ui->filter->setValue(10);
  ui->filter->setMinimum(10);

  // viewer->addCoordinateSystem(50);

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
  connect(ui->alpha, SIGNAL(valueChanged(double)), this, SLOT(TransformEdited()));
  connect(ui->filter, SIGNAL(valueChanged(int)), this, SLOT(TransformEdited()));

  if (true) {
    ui->ox->setVisible(false);
    ui->oy->setVisible(false);
    ui->oz->setVisible(false);
    ui->px->setVisible(false);
    ui->py->setVisible(false);
    ui->pz->setVisible(false);
    ui->alpha->setVisible(false);
    ui->scale->setVisible(false);
    ui->filter->setVisible(false);
    ui->oxLabel->setVisible(false);
    ui->oyLabel->setVisible(false);
    ui->ozLabel->setVisible(false);
    ui->pxLabel->setVisible(false);
    ui->pyLabel->setVisible(false);
    ui->pzLabel->setVisible(false);
    ui->GoButton->setVisible(false);
    ui->FrameEdit->setVisible(false);
    ui->BackButton->setVisible(false);
    ui->DemoButton->setVisible(false);
    ui->DisplayBox->setVisible(false);
    ui->FrameLabel->setVisible(false);
    ui->ScanIntervalLabel->setVisible(false);
    ui->PlayButton->setVisible(false);
    ui->PlaySlider->setVisible(false);
    ui->SaveButton->setVisible(false);
    ui->ScaleLabel->setVisible(false);
    ui->alphaLabel->setVisible(false);
    ui->filterLabel->setVisible(false);
    ui->DisplayButton->setVisible(false);
    ui->VideoModeButton->setVisible(false);
    ui->ScanIntervalEdit->setVisible(false);
  }
  on_DemoButton_clicked();
}

void PCLViewer::timerEvent(QTimerEvent *event) {
  if (event->timerId() != timer_id_) {
    return;
  }

  if (ui->PlayButton->text() == kPlayButtonStop) {
    auto slider = ui->PlaySlider;
//    auto& frame = gFrames[ui->PlaySlider->value()];
//    frame.alpha = ui->alpha->value();
//    frame.filter = ui->filter->value();
//    if (frame.trans) {
//      frame.Visualize(ui->VideoModeButton->text() == kScanOnly);
//      trans_ = *frame.trans;
//    } else {
//      frame.Visualize(trans_, ui->VideoModeButton->text() == kScanOnly);
//    }
    DisplayCurrentFrame();
    // gFrames[slider->value()].Visualize(ui->VideoModeButton->text() == kScanOnly);
    slider->setValue(slider->value() + 1);
  }

  ui->qvtkWidget->update();
}

void PCLViewer::TransformEdited() {
  if (gProtect) {
    return;
  }
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
    DisplayCurrentFrame();
  }
}

void PCLViewer::DisplayCurrentFrame() {
  auto& frame = gFrames[ui->PlaySlider->value()];
  frame.alpha = ui->alpha->value();
  frame.filter = ui->filter->value();
  if (frame.trans) {
    frame.Visualize(ui->VideoModeButton->text() == kScanOnly);
    trans_ = *frame.trans;
  } else {
    frame.Visualize(trans_, ui->VideoModeButton->text() == kScanOnly);
  }
  gProtect = true;
  ui->px->setValue(trans_.translation().x());
  ui->py->setValue(trans_.translation().y());
  ui->pz->setValue(trans_.translation().z());
  ui->ox->setValue(trans_.rotation().eulerAngles(0, 1, 2)[0] * 180.0 / M_PI);
  ui->oy->setValue(trans_.rotation().eulerAngles(0, 1, 2)[1] * 180.0 / M_PI);
  ui->oz->setValue(trans_.rotation().eulerAngles(0, 1, 2)[2] * 180.0 / M_PI);
  gProtect = false;

  if (frame.trans) {
    ui->DisplayBox->setChecked(true);
  } else {
    ui->DisplayBox->setChecked(false);
  }
  ui->FrameEdit->setPlaceholderText(QString::number(frame.id));

  static int buffer = 0;

  if (!ui->DemoButton->isEnabled()) {
    if (frame.id == (gFrames.size() - 1)) {
      ui->DemoButton->setEnabled(true);
    }
    if (frame.trans) {
      frame.Show();
      buffer = 10;
    } else {
      --buffer;
      if (buffer < 0) {
        frame.RemoveVisual();
      }
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
    ui->DemoButton->setEnabled(true);
  }
}

void PCLViewer::on_PlaySlider_valueChanged(int value)
{
  DisplayCurrentFrame();
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

  DisplayCurrentFrame();
}

void PCLViewer::on_ScanIntervalEdit_editingFinished()
{
  timer_id_ = startTimer(ui->ScanIntervalEdit->text().toInt());
}

void PCLViewer::on_SaveButton_clicked()
{
  std::ofstream os;
  os.open("cuts.txt");
  for (auto& frame : gFrames) {
    if (frame.trans) {
      os << frame.Write() << "\n";
    }
  }
  os.close();
}

void PCLViewer::SetFramesFromFile() {
  std::ifstream is;
  is.open("cuts.txt");
  std::string line;
  while (std::getline(is, line)) {
    std::istringstream iss(line);
    int id;
    iss >> id;
    Eigen::Affine3f::Scalar array[16];
    Eigen::Affine3f trans;
    for (int i = 0; i < 16; ++i) {
      // iss >> array[i];
      iss >> trans.data()[i];
    }
        // = Eigen::Map<Eigen::Matrix<float, 4, 4, Eigen::ColMajor>>(array);
    auto& frame = gFrames[id];
    if (frame.trans) {
      delete frame.trans;
      frame.trans = nullptr;
    }
    frame.trans = new Eigen::Affine3f;
    *frame.trans = trans;
    frame.Show();
  }
}

void PCLViewer::on_DisplayButton_clicked()
{
  auto& frame = gFrames[ui->PlaySlider->value()];
  if (frame.trans) {
    delete frame.trans;
    frame.trans = nullptr;
    ui->DisplayBox->setChecked(false);
  } else {
    frame.trans = new Eigen::Affine3f;
    *frame.trans = trans_;
    ui->DisplayBox->setChecked(true);
  }
  frame.Show();
}

void PCLViewer::on_FrameEdit_returnPressed()
{
  ui->PlaySlider->setValue(ui->FrameEdit->text().toInt());
  ui->FrameEdit->clear();
}

void PCLViewer::on_DemoButton_clicked()
{
  for (auto& frame : gFrames) {
    frame.Hide();
  }
  ui->PlaySlider->setValue(0);
  ui->DemoButton->setDisabled(true);
}
