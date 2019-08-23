#include "pclviewer.h"
#include "ui_pclviewer.h"
#include <QDir>
#include <QDebug>
#include <QTimer>
#include <frame.h>

const QString kMediaFolder = "media";
const QString kPlayButtonPlay = "Play";
const QString kPlayButtonStop = "Stop";

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

  ui->PlaySlider->setMinimum(0);
  ui->PlaySlider->setMaximum(gFrames.size() - 1);
  ui->PlaySlider->setValue(1);
//  viewer->addPointCloud (cloud, "cloud");
//  viewer->resetCamera ();
//  ui->qvtkWidget->update ();
  viewer->resetCamera();

  timer_id_ = startTimer(30);
}

void PCLViewer::timerEvent(QTimerEvent *event) {
  if (event->timerId() != timer_id_) {
    return;
  }

  if (ui->PlayButton->text() == kPlayButtonPlay) {
    auto slider = ui->PlaySlider;
    gFrames[slider->value()].Visualize();
    slider->setValue(slider->value() + 1);
  }

  ui->qvtkWidget->update();
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
  gFrames[ui->PlaySlider->value()].Visualize();
}

void PCLViewer::on_GoButton_clicked()
{
  auto slider = ui->PlaySlider;
  slider->setValue(slider->value() + 1);
  ui->PlayButton->setText(kPlayButtonStop);
}

void PCLViewer::on_BackButton_clicked()
{
  auto slider = ui->PlaySlider;
  slider->setValue(slider->value() - 1);
  ui->PlayButton->setText(kPlayButtonStop);
}
