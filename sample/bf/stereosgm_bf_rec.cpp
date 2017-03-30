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
#include <opencv2/opencv.hpp>
#include <string>

#include <libsgm.h>

#include <atomic>
#include "bf.hpp"
#include "demo.h"
#include "renderer.h"

using namespace std;

std::atomic_bool running;
void intHandler(int signo) { running.store(false); }

cv::Mat depthmapColorEncoding(cv::Mat &map) {
  double min;
  double max;
  cv::minMaxIdx(map, &min, &max);
  cv::Mat adjMap;
  // expand your range to 0..255. Similar to histEq();
  map.convertTo(adjMap, CV_8UC1, 255 / (max - min), -min);

  // this is great. It converts your grayscale image into a tone-mapped one,
  // much more pleasing for the eye
  // function is found in contrib module, so include contrib.hpp
  // and link accordingly

  cv::Mat map8u;
  cv::normalize(map, map8u, 255, 0, cv::NORM_MINMAX, CV_8U);

  cv::Mat falseColorsMap;
  applyColorMap(map8u, falseColorsMap, cv::COLORMAP_OCEAN);

  return falseColorsMap;
}

bool saveAsPLY(vector<cv::Vec3f> Points, vector<cv::Vec3b> colors,
               const char *fileName) {
  FILE *fid1 = NULL;
#ifdef WIN32
  fopen_s(&fid1, fileName, "wb");
#else
  fid1 = fopen(fileName, "wb");
#endif

  if (fid1 == NULL) {
    printf("unable to save output PLY file as %s\n", fileName);
    return false;
  }

  std::fprintf(fid1, "ply\n");
  fprintf(fid1, "format ascii 1.0\n");

  fprintf(fid1, "element vertex %lu\n", Points.size());
  fprintf(fid1, "property float x\n");
  fprintf(fid1, "property float y\n");
  fprintf(fid1, "property float z\n");
  fprintf(fid1, "property uchar red\n");
  fprintf(fid1, "property uchar green\n");
  fprintf(fid1, "property uchar blue\n");

  int faceNum = (int)Points.size() - 2 >= 0 ? (int)Points.size() - 2 : 0;
  fprintf(fid1, "element face %lu\n", faceNum);
  fprintf(fid1, "property list uchar int vertex_indices\n");

  fprintf(fid1, "end_header\n");

  for (int i = 0; i < Points.size(); i++) {
    fprintf(fid1, "%f %f %f %d %d %d\n", Points[i][0], Points[i][1],
            Points[i][2], colors[i][2], colors[i][1], colors[i][0]);
  }

  for (int i = 0; i < faceNum; i++) {
    fprintf(fid1, "3 %d %d %d\n", i, i + 1, i + 2);
  }

  fclose(fid1);
  return true;
}

cv::Mat getImageFromBF(bf::Subscriber &subscriber, double &ts,
                       timespec &tspec) {
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

    ts = lastest_datum.time.tv_sec + lastest_datum.time.tv_nsec / 1000000000.0;
    tspec = lastest_datum.time;
    free(lastest_datum.data);
  }

  return image;
}

int main(int argc, char *argv[]) {
  if (argc < 2) {
    std::cerr << "usage: stereosgm <stereo_img_publisher> "
                 "<disparity_size> <stereo_calib_file> <baseline_in_meter> "
                 "<bgrd_msg> <qmat_msg>"
              << std::endl;
    std::exit(EXIT_FAILURE);
  }

  signal(SIGINT, intHandler);

  //  bf::Subscriber left_subscriber(argv[1], "master");
  //  bf::Subscriber right_subscriber(argv[2], "master");

  bf::Subscriber stereo_subscriber(argv[1], "master");
  bf::Publisher bgrd_publisher(argv[5], "master");
  bf::Publisher qmat_publisher(argv[6], "master");

  int disp_size = atoi(argv[2]);

  double baseline = atof(argv[4]);

  cv::Mat left_gray, right_gray;
  cv::Mat left_color_rectified, left_gray_rectified, right_gray_rectified;
  //  do {
  //    left = getImageFromBF(left_subscriber);
  //    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  //  } while (left.empty());
  //
  //  int width = left.cols;
  //  int height = left.rows;

  cv::FileStorage fs(argv[3], cv::FileStorage::READ);
  cv::Mat K_left, K_right, dist_coeffs_left, dist_coeffs_right, R, t;
  cv::Mat R1, R2, P1, P2, Q;
  int img_w, img_h;
  cv::Rect validRoi[2];

  fs["image_width"] >> img_w;
  fs["image_height"] >> img_h;
  fs["K_left"] >> K_left;
  fs["K_right"] >> K_right;
  fs["dist_coeffs_left"] >> dist_coeffs_left;
  fs["dist_coeffs_right"] >> dist_coeffs_right;
  fs["R"] >> R;
  fs["t"] >> t;

  t = t / norm(t) * baseline;

  //  cv::Mat R_tmp, rvec(3, 1, CV_64FC1, cv::Scalar(0));
  //  rvec.at<double>(1) = -3.14 / 300;
  //  cv::Rodrigues(rvec, R_tmp);
  //  R = R_tmp * R;

  int s = 2;

  cv::stereoRectify(K_left, dist_coeffs_left, K_right, dist_coeffs_right,
                    cv::Size(img_w, img_h), R, t, R1, R2, P1, P2, Q,
                    CV_CALIB_ZERO_DISPARITY, 0, cv::Size(img_w / s, img_h / s),
                    &validRoi[0], &validRoi[1]);

  img_w = img_w / s;
  img_h = img_h / s;

  cout << "Q:" << endl << Q << endl;
  //  cout << "R1: " << endl << R1 << endl;
  //  cout << "R2: " << endl << R2 << endl;
  //  cout << "P1: " << endl << P1 << endl;
  //  cout << "P2: " << endl << P2 << endl;

  cv::Mat left_map[2], right_map[2];
  initUndistortRectifyMap(K_left, dist_coeffs_left, R1,
                          P1(cv::Rect(0, 0, 3, 3)), cv::Size(img_w, img_h),
                          CV_16SC2, left_map[0], left_map[1]);
  initUndistortRectifyMap(K_right, dist_coeffs_right, R2,
                          P2(cv::Rect(0, 0, 3, 3)), cv::Size(img_w, img_h),
                          CV_16SC2, right_map[0], right_map[1]);

  //  cv::Mat v4(4, 1, CV_64FC1);
  //  v4.at<double>(0) = 1446;
  //  v4.at<double>(1) = 735;
  //  v4.at<double>(2) = 23;
  //  v4.at<double>(3) = 1;
  //  cv::Mat output = Q * v4;
  //  output /= output.at<double>(3);
  //  cout << output << endl;

  cudaGLSetGLDevice(0);

  SGMDemo demo(img_w, img_h);
  if (demo.init()) {
    printf("fail to init SGM Demo\n");
    std::exit(EXIT_FAILURE);
  }

  sgm::StereoSGM ssgm(img_w, img_h, disp_size, 8, 16,
                      sgm::EXECUTE_INOUT_HOST2CUDA);

  Renderer renderer(img_w, img_h);

  uint16_t *d_output_buffer = NULL;

  cv::namedWindow("stereo", cv::WINDOW_NORMAL);
  cv::namedWindow("left_gray_rectified", cv::WINDOW_NORMAL);
  cv::namedWindow("right_gray_rectified", cv::WINDOW_NORMAL);
  cv::Mat color_depth_map(img_h, img_w, CV_8UC3);
  // int frame_no = 0;
  running.store(true);
  bool continuous_saving = false;
  int out_cnt = 0;
  while (running.load()) {
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT);

    //    double left_ts, right_ts;
    //    cv::Mat left_color = getImageFromBF(left_subscriber, left_ts);
    //    cv::Mat right_color = getImageFromBF(right_subscriber, right_ts);

    double stereo_ts;
    timespec tp;
    cv::Mat stereo_color = getImageFromBF(stereo_subscriber, stereo_ts, tp);

    if (!stereo_color.empty()) {
      //////////////////////////////////////////////////////////////////////////////////
      //      cv::Mat left_orig = stereo_color(
      //                  cv::Rect(0, 0, stereo_color.cols / 2,
      //                  stereo_color.rows)),
      //              right_orig = stereo_color(cv::Rect(stereo_color.cols / 2,
      //              0,
      //                                                 stereo_color.cols / 2,
      //                                                 stereo_color.rows));
      //
      //      cv::Mat left_center =
      //                  left_orig(cv::Rect(left_orig.cols / 4, left_orig.rows
      //                  / 4,
      //                                     left_orig.cols / 2, left_orig.rows
      //                                     / 2)),
      //              right_center = right_orig(
      //                  cv::Rect(right_orig.cols / 4 - 50, right_orig.rows /
      //                  4,
      //                           right_orig.cols / 2, right_orig.rows / 2));
      //
      //      stereo_color = cv::Mat(left_center.rows, left_center.cols * 2,
      //      CV_8UC3);
      //      left_center.copyTo(
      //          stereo_color(cv::Rect(0, 0, left_center.cols,
      //          left_center.rows)));
      //      right_center.copyTo(stereo_color(
      //          cv::Rect(left_center.cols, 0, left_center.cols,
      //          left_center.rows)));

      //////////////////////////////////////////////////////////////////////////////////

      //      cvtColor(stereo_color(
      //                   cv::Rect(0, 0, stereo_color.cols / 2,
      //                   stereo_color.rows)),
      //               left_gray, CV_BGR2GRAY);
      //      remap(left_gray, left_gray_rectified, left_map[0], left_map[1],
      //            cv::INTER_LINEAR);
      remap(stereo_color(
                cv::Rect(0, 0, stereo_color.cols / 2, stereo_color.rows)),
            left_color_rectified, left_map[0], left_map[1], cv::INTER_LINEAR);
      cvtColor(left_color_rectified, left_gray_rectified, CV_BGR2GRAY);

      cvtColor(stereo_color(cv::Rect(stereo_color.cols / 2, 0,
                                     stereo_color.cols / 2, stereo_color.rows)),
               right_gray, CV_BGR2GRAY);
      remap(right_gray, right_gray_rectified, right_map[0], right_map[1],
            cv::INTER_LINEAR);

      ssgm.execute(
          left_gray_rectified.data, right_gray_rectified.data,
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

      cv::Mat full_size_disp(left_gray_rectified.rows, left_gray_rectified.cols,
                             CV_16SC1);
      cudaMemcpy(full_size_disp.data, d_output_buffer,
                 sizeof(uint16_t) * left_gray_rectified.rows *
                     left_gray_rectified.cols,
                 cudaMemcpyDeviceToHost);
      cv::Mat disp_8u;
      full_size_disp.convertTo(disp_8u, CV_8U, 1, -24 / s);
      cv::Mat in[] = {left_color_rectified, disp_8u};
      cv::Mat bgrd(left_color_rectified.size(), CV_8UC4);
      int from_to[] = {0, 0, 1, 1, 2, 2, 3, 3};
      mixChannels(in, 2, &bgrd, 1, from_to, 4);
      bgrd_publisher.publish(bgrd.data, bgrd.size().width, bgrd.size().height,
                             bgrd.elemSize(), bgrd.type(), &tp);
      qmat_publisher.publish(Q.data, Q.size().width, Q.size().height,
                             Q.elemSize(), Q.type(), &tp);

      renderer.render_disparity_color(d_output_buffer, disp_size,
                                      color_depth_map.data);
      imshow("stereo", stereo_color);
      imshow("left_gray_rectified", left_gray_rectified);
      imshow("right_gray_rectified", right_gray_rectified);
      demo.swap_buffer();

      char c = cv::waitKey(10);

      // save intermediate results
      if (c == 's') {
        cv::Mat ds_disp;

        double ds = 0.1;

        cv::resize(full_size_disp, ds_disp, cv::Size(), ds, ds,
                   cv::INTER_NEAREST);

        cv::Mat ds_Q = Q.clone();
        ds_Q.col(0) = ds_Q.col(0) / ds;
        ds_Q.col(1) = ds_Q.col(1) / ds;

        cv::Mat pt_cloud;
        reprojectImageTo3D(ds_disp, pt_cloud, ds_Q, true);

        // cv::Mat left_color_rectified;
        // remap(stereo_color(
        //          cv::Rect(0, 0, stereo_color.cols / 2, stereo_color.rows)),
        //      left_color_rectified, left_map[0], left_map[1],
        //      cv::INTER_LINEAR);
        cv::Mat ds_left_color_rectified;
        cv::resize(left_color_rectified, ds_left_color_rectified, cv::Size(),
                   ds, ds);

        vector<cv::Vec3f> pts;
        vector<cv::Vec3b> colors;
        for (int i = 0; i < pt_cloud.rows; i++) {
          for (int j = 0; j < pt_cloud.cols; j++) {
            cv::Vec3f pt = pt_cloud.at<cv::Vec3f>(i, j);
            if (pt[2] < 10000) {
              pts.push_back(pt);
              colors.push_back(ds_left_color_rectified.at<cv::Vec3b>(i, j));
            }
          }
        }

        saveAsPLY(pts, colors, "ptcloud.ply");
        cv::imwrite("left.jpg", left_gray_rectified);
        cv::imwrite("right.jpg", right_gray_rectified);
        cout << "Q:" << endl << Q << endl;
        cout << "ds_Q:" << endl << ds_Q << endl;

        cout << "point cloud saved." << endl;

        cv::Mat full_pt_cloud;
        reprojectImageTo3D(full_size_disp, full_pt_cloud, Q, true);
        vector<cv::Vec3f> full_pts;
        vector<cv::Vec3b> full_colors;
        for (int i = 0; i < full_pt_cloud.rows; i++) {
          for (int j = 0; j < full_pt_cloud.cols; j++) {
            cv::Vec3f pt = full_pt_cloud.at<cv::Vec3f>(i, j);
            if (pt[2] < 10000) {
              full_pts.push_back(pt);
              full_colors.push_back(left_color_rectified.at<cv::Vec3b>(i, j));
            }
          }
        }
        saveAsPLY(full_pts, full_colors, "full_ptcloud.ply");

        // cv::Mat dmap = depthmapColorEncoding(full_size_disp);
        cv::flip(color_depth_map, color_depth_map, 0);
        cv::imwrite("depth.jpg", color_depth_map);
      }

      if (c == 'c') continuous_saving = !continuous_saving;

      // save intermediate results
      if (continuous_saving) {
        char name[256];
        sprintf(name, "color%04d.png", out_cnt);
        imwrite(name, left_color_rectified);
        sprintf(name, "disp%04d.png", out_cnt++);
        imwrite(name, disp_8u);

        cv::FileStorage fs("Q.xml", cv::FileStorage::WRITE);
        fs << "Q" << Q;

        cout << out_cnt << endl;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}
