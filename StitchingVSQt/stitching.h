#ifndef STITCHING_H
#define STITCHING_H
#include <iostream>
using std::string;
void storeImageFileName(string name);
int startCalculation();
bool stitch(std::vector<std::vector<string>>imageNames, std::vector<int>center_idx);
//Mat pano_image();


#endif