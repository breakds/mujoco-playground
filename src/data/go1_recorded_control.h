#pragma once

#include <vector>

#include "mujoco/mujoco.h"

namespace mpg {

std::vector<mjtNum> GetRecordedControls();

}  // namespace mpg
