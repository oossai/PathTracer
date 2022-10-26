#pragma once

#if defined(_MSC_VER)
#define NOMINMAX
#pragma once
#endif

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>

#include <iostream>
#include <limits>
#include <vector>
#include <array>
#include <string>
#include <string_view>
#include <fstream>
#include <algorithm>
#include <functional>
#include <random>
#include <memory>
#include <numeric>
#include <numbers>
#include <stdexcept>

using Float = float;
