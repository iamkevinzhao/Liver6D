#ifndef FRAME_H
#define FRAME_H

#include <QPixmap>
#include <vector>
#include <QLabel>

class Frame
{
public:
  QPixmap image;
  QLabel* label = nullptr;

  Frame();
  void Visualize();
};

extern std::vector<Frame> gFrames;

#endif // FRAME_H
