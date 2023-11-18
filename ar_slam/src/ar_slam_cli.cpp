/*
Copyright (c) 2023 Derek King

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/


#include <iostream>
#include <string>
#include <vector>

#include "ar_slam/ar_slam_util.hpp"

#include "glog/logging.h"


int main(int argc, char ** argv)
{
  google::InitGoogleLogging(argv[0]);

  if (argc < 2) {
    std::cerr << "Need to provide a .yaml or list of *.jpg for processing" << std::endl;
    std::cerr <<
      "Usage:  slam [fn1.yaml] [fn2.jpg] [fn3.jpg] ..."
      "\n"
      "Description run slam on pre-processes detection and/or images\n"
      "\n"
      "Processing images to find aruco tags is relatively slow, so this can run with \n"
      "yaml input as input instead of imsage with stand-alone detections or outputs from previous mapping runs\n"
      "\n"
      "Example 1: just perform slam with images\n"
      "  ar_slam slam img1.jpg img2.jpg img3.jpg\n"
      "\n"
      "Example 2: re-run slam with just saved map\n"
      "  ar_slam slam map.yaml\n"
      "\n"
      "Example 3: re-run slam with just saved map and a few more images\n"
      "  ar_slam slam map.yaml img1.png img2.png\n";
    return 1;
  }

  ArSlamSolver solver;
  std::vector<std::string> img_fns;
  for (int ii = 1; ii < argc; ++ii) {
    std::string fn = argv[ii];
    if (endswith(fn, ".yaml")) {
      solver.loadYaml(fn);
    } else {
      img_fns.emplace_back(argv[ii]);
    }
  }
  solver.loadImages(img_fns);

  solver.solve();
  solver.printCameras();

  {
    std::string fn = "map.yaml";
    std::cout << "Saving results to " << fn << std::endl;
    std::ofstream file(fn);
    solver.saveYaml(file);
  }

  return 0;
}
