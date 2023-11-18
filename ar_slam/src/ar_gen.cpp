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

#include <unistd.h>

// OpenCV
#include "opencv2/opencv.hpp"
#include "opencv2/aruco.hpp"


int main()
{
  cv::namedWindow("Image");
  usleep(50 * 1000);

  // 8.5 x 11
  // margin of 1.0 = 6.5 x 9
  // 3x2 pattern
  // 0.5 in pad (2 x 1)
  // pad = 6.0 x 8
  // 6.0 / 2 = 3.0
  // 8.5 / 3 = 2.6

  // Margin
  // use 2.5 x 2.5 for markers
  // 250 x 250
  // 850 x 1000

  const float dpi = 100;
  const float margin = 0.5 * dpi;
  const float w = 8.5 * dpi - margin;
  const float h = 11.0 * dpi - margin;
  const float ar_size = 2.5 * dpi;
  const float pad = 1.0 * dpi;

  const int max_ar_idx = 50;
  // cv::aruco::DICT_6X6_250
  cv::Ptr<cv::aruco::Dictionary> dictionary = cv::aruco::getPredefinedDictionary(
    cv::aruco::DICT_4X4_50);

  const int xcnt = 2;
  const int ycnt = 3;

  const float xcenter = 0.5 * w;
  const float ycenter = 0.5 * h;

  int ar_idx = 0;
  int page_idx = 0;
  while (ar_idx < max_ar_idx) {
    cv::Mat img(h, w, CV_8UC1, cv::Scalar(255));

    for (int xi = 0; xi < xcnt; ++xi) {
      for (int yi = 0; yi < ycnt; ++yi) {
        if (ar_idx >= max_ar_idx) {
          break;
        }

        float xoff = float(xi - 0.5f * (xcnt - 1)) * (ar_size + pad);
        float yoff = float(yi - 0.5f * (ycnt - 1)) * (ar_size + pad);
        int xcorner = (xcenter - 0.5 * ar_size + xoff);
        int ycorner = (ycenter - 0.5 * ar_size + yoff);

        cv::Rect roi(xcorner, ycorner, ar_size, ar_size);
        cv::Mat slice = img(roi);
        cv::aruco::drawMarker(dictionary, ar_idx, ar_size, slice, 1);
        cv::putText(
          img, std::to_string(ar_idx),
          cv::Point(xcorner + 50, ycorner - 15),
          cv::FONT_HERSHEY_DUPLEX, 0.5, cv::Scalar(100), 1);
        ++ar_idx;
      }
    }

    ++page_idx;
    std::ostringstream fn;
    fn << "aruco4x4_50_page" << page_idx << ".png";
    cv::imwrite(fn.str(), img);
    cv::imshow("Image", img);
    cv::waitKey(200);
  }

  return 0;
}
