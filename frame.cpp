#include "frame.h"

Frame::Frame()
{

}

void Frame::Visualize() {
  if (label) {
    label->setPixmap(image);
  }
}

std::vector<Frame> gFrames;
