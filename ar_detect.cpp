/*
Copyright 2023 Derek King

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice,
   this list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS “AS IS” AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

#include <iostream>
#include <string>
#include <vector>

#include "ar_slam_util.hpp"

#include "glog/logging.h"


int main(int argc, char** argv)
{
  google::InitGoogleLogging(argv[0]);

  if (argc < 2)
  {
    std::cerr << "Need to provide list of *.jpg for processing" << std::endl;
    std::cerr <<
      "Usage: ar_detect img1.jpg [img2.jpg] ..."
      "\n"
      "Description pre-processes image captures for Aruco tags\n"
      "\n"
      "Processing images to find aruco tags is relatively slow, so this can run\n"
      "  with pre-processes detections instead images\n";
    return 1;
  }

  std::vector<std::string> img_fns;
  for (int ii = 1; ii<argc; ++ii)
  {
    img_fns.emplace_back(argv[ii]);
  }
  ArSlamSolver solver;
  solver.loadImages(img_fns);

  {
    std::string fn = "detect.yaml";
    std::cout << "Saving results to " << fn << std::endl;
    std::ofstream file(fn);
    solver.saveYaml(file);
  }

  return 0;
}
