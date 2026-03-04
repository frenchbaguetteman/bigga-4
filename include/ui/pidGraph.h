#pragma once

namespace PIDGraphUI {

void init();
void addSample(double error, double output);
void draw(int x0, int y0, int x1, int y1);

}  // namespace PIDGraphUI
