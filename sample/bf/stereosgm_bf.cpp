/*
Copyright 2016 fixstars

Licensed under the Apache License, Version 2.0 (the "License");
you may not use this file except in compliance with the License.
You may obtain a copy of the License at

http ://www.apache.org/licenses/LICENSE-2.0

Unless required by applicable law or agreed to in writing, software
distributed under the License is distributed on an "AS IS" BASIS,
WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
See the License for the specific language governing permissions and
limitations under the License.
*/

#include <stdlib.h>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <string>

#include <libsgm.h>

#include <atomic>
#include "bf.hpp"
#include "demo.h"
#include "renderer.h"

using namespace std;

std::atomic_bool running;
void intHandler(int signo) { running.store(false); }

cv::Mat getImageFromBF(bf::Subscriber &subscriber) {
  cv::Mat image;

  bf::Datum lastest_datum = subscriber.get_the_lastest_and_clear_all();
  if (lastest_datum.size != 0 && lastest_datum.data != NULL) {
    if (lastest_datum.data_type == kDATA_TYPE_MATRIX) {
      image = cv::Mat(lastest_datum.height, lastest_datum.width,
                      lastest_datum.matrix_type, lastest_datum.data);
      image = image.clone();
    } else {
      std::vector<char> buf(
          (unsigned char *)lastest_datum.data,
          (unsigned char *)lastest_datum.data + lastest_datum.size);
      image = cv::imdecode(buf, CV_LOAD_IMAGE_COLOR);
    }

    free(lastest_datum.data);
  }

  return image;
}

int main(int argc, char *argv[]) {
  if (argc < 2) {
    std::cerr << "usage: stereosgm <left_img_publisher> <right_img_publisher> "
                 "<disparity_size> "
              << std::endl;
    std::exit(EXIT_FAILURE);
  }

  signal(SIGINT, intHandler);

  bf::Subscriber left_subscriber(argv[1], "master");
  bf::Subscriber right_subscriber(argv[2], "master");

  int disp_size = atoi(argv[3]);

  cv::Mat left, right;
  do {
    left = getImageFromBF(left_subscriber);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  } while (left.empty());

  int width = left.cols;
  int height = left.rows;

  cudaGLSetGLDevice(0);

  SGMDemo demo(width, height);
  if (demo.init()) {
    printf("fail to init SGM Demo\n");
    std::exit(EXIT_FAILURE);
  }

  sgm::StereoSGM ssgm(width, height, disp_size, 8, 16,
                      sgm::EXECUTE_INOUT_HOST2CUDA);

  Renderer renderer(width, height);

  uint16_t *d_output_buffer = NULL;

  cv::namedWindow("left", cv::WINDOW_NORMAL);
  // int frame_no = 0;
  running.store(true);
  while (running.load()) {
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT);

    cv::Mat left_color = getImageFromBF(left_subscriber);
    cv::Mat right_color = getImageFromBF(right_subscriber);

    if (!left_color.empty() && !right_color.empty()) {
      cvtColor(left_color, left, CV_BGR2GRAY);
      cvtColor(right_color, right, CV_BGR2GRAY);

      ssgm.execute(
          left.data, right.data,
          (void **)&d_output_buffer);  // , sgm::DST_TYPE_CUDA_PTR, 16);

      //    switch (demo.get_flag()) {
      //      case 0: {
      //        renderer.render_input((uint16_t *)left.data);
      //      } break;
      //      case 1:
      //        renderer.render_disparity(d_output_buffer, disp_size);
      //        break;
      //      case 2:
      //        renderer.render_disparity_color(d_output_buffer, disp_size);
      //        break;
      //    }

      renderer.render_disparity_color(d_output_buffer, disp_size, NULL);
      imshow("left", left_color);
      demo.swap_buffer();

      cv::waitKey(10);
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}
