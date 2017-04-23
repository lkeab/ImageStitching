#ifndef STITCHING_H
#define STITCHING_H
#include <iostream>
using std::string;

int startCalculation();
bool stitch(std::vector<std::vector<string>>imageNames, std::vector<int>center_idx);

#endif