#ifndef FRAME_H
#define FRAME_H

#include <QPixmap>
#include <vector>

class Frame
{
public:
  QPixmap image;
  Frame();
};

extern std::vector<Frame> gFrames;

#endif // FRAME_H
