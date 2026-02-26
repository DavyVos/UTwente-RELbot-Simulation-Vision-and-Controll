#ifndef PRESETS_H
#define PRESETS_H
#include <tuple>

using Point = std::tuple<double, double>;

// Individual shapes
extern const Point circle[];
extern const Point figureEight[];
extern const Point somethingAgon[];
extern const Point straightline[];

// Preset collection
extern const Point* presets[];

#endif