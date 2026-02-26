#include "presets.h"

using namespace std;

const tuple<double, double> circle[] = {
    tuple(1.0, 2.0),
    tuple(1.0, 2.0),
    tuple(1.0, 2.0),
    tuple(1.0, 2.0),
};

const tuple<double, double> figureEight[] = {
    tuple(1.0, 2.0),
    tuple(1.0, 2.0),
    tuple(2.0, 1.0),
    tuple(2.0, 1.0),
};

const tuple<double, double> somethingAgon[] = {
    tuple(1.0, 1.2),
    tuple(1.0, 1.0),
    tuple(1.0, 1.2),
    tuple(1.0, 1.0),
};

const tuple<double, double> straightline[] = {
    tuple(0.0, 0.0),
    tuple(5.0, 5.0),
    tuple(1.0, 1.0),
    tuple(6.0, 6.0),
};

const tuple<double, double>* presets[] = {
    circle,
    figureEight,
    somethingAgon,
    straightline
};