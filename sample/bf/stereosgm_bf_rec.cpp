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

#define PI 3.1415926

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

// any plane not close to appx_angle will not be tested again all pts, just
// continue with next trial
void ransacPlaneFitting(vector<cv::Point3f> &pts, int iters,
                        double angle_thresh, double dist_thresh,
                        cv::Vec4d &plane, vector<uchar> &in_flags,
                        int &num_inliers) {
  in_flags = vector<uchar>(pts.size(), 0);
  num_inliers = 0;
  for (int i = 0; i < iters; i++) {
    std::set<int> indices;
    while (indices.size() < 3) {
      int idx = std::rand() % pts.size();
      if (indices.find(idx) == indices.end()) indices.insert(idx);
    }

    cv::Mat M(3, 3, CV_64FC1), b(3, 1, CV_64FC1, cv::Scalar(-1));
    std::set<int>::iterator it = indices.begin();
    M.at<double>(0, 0) = pts[*it].x;
    M.at<double>(0, 1) = pts[*it].y;
    M.at<double>(0, 2) = pts[*it].z;
    it++;

    M.at<double>(1, 0) = pts[*it].x;
    M.at<double>(1, 1) = pts[*it].y;
    M.at<double>(1, 2) = pts[*it].z;
    it++;

    M.at<double>(2, 0) = pts[*it].x;
    M.at<double>(2, 1) = pts[*it].y;
    M.at<double>(2, 2) = pts[*it].z;

    cv::Mat x = M.inv() * b;   // x.dot(p)+1=0 is the plane
    cv::Mat n = x / norm(x);   // plane normal
    double d = 1.0 / norm(x);  // plane becomes n.dot(p)+d=0;

    cv::Point3d up_dir(0, -1, 0);
    cv::Point3d current_plane_normal(n);
    double angle_to_up = acos(up_dir.dot(current_plane_normal));

    // cout << current_plane_normal << endl;

    // too close to vertical direction
    if (angle_to_up > angle_thresh && angle_to_up < PI - angle_thresh) {
      vector<uchar> inlier_flags(pts.size(), 0);
      int cur_num_inliers = 0;
      for (int k = 0; k < pts.size(); k++) {
        double dist = fabs(current_plane_normal.dot(
                               cv::Point3d(pts[k].x, pts[k].y, pts[k].z)) +
                           d);
        // cout << dist << endl;
        if (dist < dist_thresh) {
          cur_num_inliers++;
          inlier_flags[k] = 1;
        }
      }

      if (cur_num_inliers > num_inliers) {
        num_inliers = cur_num_inliers;
        plane[0] = current_plane_normal.x;
        plane[1] = current_plane_normal.y;
        plane[2] = current_plane_normal.z;
        plane[3] = d;

        in_flags = inlier_flags;
      }
    }
  }

  //  // use all inliers to recompute plane
  //  cv::Mat M(max_inliers, 3, CV_64FC1),
  //      b(max_inliers, 1, CV_64FC1, cv::Scalar(-1));
  //  int cnt = 0;
  //  for (int i = 0; i < in_flags.size(); i++) {
  //    if (in_flags[i]) {
  //      M.at<double>(cnt, 0) = pts[i].x;
  //      M.at<double>(cnt, 1) = pts[i].y;
  //      M.at<double>(cnt, 2) = pts[i].z;
  //      cnt++;
  //    }
  //  }
  //  cv::Mat x = M.inv(cv::DECOMP_SVD) * b;  // x.dot(p)+1=0 is the plane
  //  cv::Mat n = x / norm(x);                // plane normal
  //  double d = 1.0 / norm(x);               // plane becomes n.dot(p)+d=0;
  //
  //  plane[0] = n.at<double>(0);
  //  plane[1] = n.at<double>(1);
  //  plane[2] = n.at<double>(2);
  //  plane[3] = d;
}

void computeVDisparity(cv::Mat &disp_map, cv::Mat &v_disp) {
  v_disp = cv::Mat(disp_map.size().height, 128, CV_32SC1, cv::Scalar(0));
  for (int i = 0; i < disp_map.rows; i++) {
    for (int j = 0; j < disp_map.cols; j++) {
      int d = disp_map.at<uchar>(i, j);
      if (d > 0 && d < 128) {
        v_disp.at<int>(i, d)++;
      }
    }
  }
}

void connectedComponentsVPlane(cv::Mat &disp_map, cv::Mat &color_labeling,
                               cv::Mat &Q, cv::Mat &topview) {
  int tv_w = 500, tv_h = 500;
  double ppm = 5;
  double top_x = -tv_w / 5 / 2;
  double top_z = tv_w / 5 / 2;
  topview = cv::Mat(tv_h, tv_w, CV_8UC3, cv::Scalar(0, 0, 0));

  cv::Mat local_disp_map = disp_map.clone();

  cv::cvtColor(disp_map, color_labeling, CV_GRAY2RGB);
  cv::Mat visited(disp_map.size(), CV_8UC1, cv::Scalar(0));

  vector<cv::Point> pt_queue(disp_map.rows * disp_map.cols);
  int queue_len = 0;
  int w = disp_map.cols, h = disp_map.rows;

  for (int i = 0; i < disp_map.rows; i++) {
    for (int j = 0; j < disp_map.cols; j++) {
      if (disp_map.at<uchar>(i, j) > 0 && visited.at<uchar>(i, j) == 0) {
        uchar d = disp_map.at<uchar>(i, j);
        pt_queue[0] = cv::Point(j, i);
        queue_len = 1;
        int head_ptr = 0;
        visited.at<uchar>(pt_queue[0]) = 1;

        cv::Point lm(disp_map.cols, 0), rm(0, 0), bm(0, 0),
            tm(0, disp_map.rows);
        while (head_ptr < queue_len) {
          cv::Point &p = pt_queue[head_ptr];
          head_ptr++;

          if (p.x < lm.x) lm = p;
          if (p.x > rm.x) rm = p;
          if (p.y > bm.y) bm = p;
          if (p.y < tm.y) tm = p;

          if (p.x > 0 && p.x < w - 1 && p.y > 0 && p.y < h - 1) {
            int u, v;

            u = p.x - 1, v = p.y - 1;
            if (disp_map.at<uchar>(v, u) == d && visited.at<uchar>(v, u) == 0) {
              pt_queue[queue_len] = cv::Point(u, v);
              visited.at<uchar>(v, u) = 1;
              queue_len++;
            }

            u = p.x, v = p.y - 1;
            if (disp_map.at<uchar>(v, u) == d && visited.at<uchar>(v, u) == 0) {
              pt_queue[queue_len] = cv::Point(u, v);
              visited.at<uchar>(v, u) = 1;
              queue_len++;
            }

            u = p.x + 1, v = p.y - 1;
            if (disp_map.at<uchar>(v, u) == d && visited.at<uchar>(v, u) == 0) {
              pt_queue[queue_len] = cv::Point(u, v);
              visited.at<uchar>(v, u) = 1;
              queue_len++;
            }

            u = p.x - 1, v = p.y;
            if (disp_map.at<uchar>(v, u) == d && visited.at<uchar>(v, u) == 0) {
              pt_queue[queue_len] = cv::Point(u, v);
              visited.at<uchar>(v, u) = 1;
              queue_len++;
            }

            u = p.x + 1, v = p.y;
            if (disp_map.at<uchar>(v, u) == d && visited.at<uchar>(v, u) == 0) {
              pt_queue[queue_len] = cv::Point(u, v);
              visited.at<uchar>(v, u) = 1;
              queue_len++;
            }

            u = p.x - 1, v = p.y + 1;
            if (disp_map.at<uchar>(v, u) == d && visited.at<uchar>(v, u) == 0) {
              pt_queue[queue_len] = cv::Point(u, v);
              visited.at<uchar>(v, u) = 1;
              queue_len++;
            }

            u = p.x, v = p.y + 1;
            if (disp_map.at<uchar>(v, u) == d && visited.at<uchar>(v, u) == 0) {
              pt_queue[queue_len] = cv::Point(u, v);
              visited.at<uchar>(v, u) = 1;
              queue_len++;
            }

            u = p.x + 1, v = p.y + 1;
            if (disp_map.at<uchar>(v, u) == d && visited.at<uchar>(v, u) == 0) {
              pt_queue[queue_len] = cv::Point(u, v);
              visited.at<uchar>(v, u) = 1;
              queue_len++;
            }
          }
        }

        // double hw_ratio = (double)(bm.y - tm.y + 1) / (rm.x - lm.x);
        if (queue_len > 300 && bm.y - tm.y + 1 > 20) {
          cv::Vec3b c(std::rand() % 200 + 55, std::rand() % 200 + 55,
                      std::rand() % 200 + 55);
          c = cv::Vec3b(0, 255, 0);
          for (int k = 0; k < queue_len; k++) {
            color_labeling.at<cv::Vec3b>(pt_queue[k]) = c;
            local_disp_map.at<uchar>(pt_queue[k]) = 0;
          }

          // draw top view
          cv::Mat vl = (cv::Mat_<double>(4, 1) << lm.x, lm.y, d, 1);
          cv::Mat vr = (cv::Mat_<double>(4, 1) << rm.x, rm.y, d, 1);
          cv::Mat vb = (cv::Mat_<double>(4, 1) << bm.x, bm.y, d, 1);
          cv::Mat p3l = Q * vl, p3r = Q * vr, p3b = Q * vb;
          p3l /= p3l.at<double>(3);
          p3r /= p3r.at<double>(3);
          p3b /= p3b.at<double>(3);

          if (p3b.at<double>(1) > -1) {
            double lx = (p3l.at<double>(0) - top_x) * ppm,
                   ly = (top_z - p3l.at<double>(2)) * ppm;
            double rx = (p3r.at<double>(0) - top_x) * ppm,
                   ry = (top_z - p3r.at<double>(2)) * ppm;

            //          cout << lm.x << ", " << lm.y << "====" << lx << ", " <<
            //          ly
            //          << endl;
            //          cout << rm.x << ", " << rm.y << "====" << rx << ", " <<
            //          ry
            //          << endl
            //               << endl;

            cv::line(topview, cv::Point(lx, ly), cv::Point(rx, ry),
                     cv::Scalar(255, 255, 255), 2);
            cv::circle(topview, cv::Point(topview.cols / 2, topview.rows / 2),
                       2, cv::Scalar(0, 255, 0), -1);
          }
        }
      }
    }
  }

  // find connected components that is vertical plane
  //  cv::Mat pt_cloud;
  //  reprojectImageTo3D(local_disp_map, pt_cloud, Q, true);
  //  double angle_thresh = 60.f / 180.f * PI;
  //  cv::cvtColor(local_disp_map, color_labeling, CV_GRAY2RGB);
  //  color_labeling *= 2;
  //
  //  visited = 0;
  //  for (int i = 0; i < local_disp_map.rows; i++) {
  //    for (int j = 0; j < local_disp_map.cols; j++) {
  //      if (local_disp_map.at<uchar>(i, j) > 0 && visited.at<uchar>(i, j) ==
  //      0) {
  //        pt_queue[0] = cv::Point(j, i);
  //        queue_len = 1;
  //        int head_ptr = 0;
  //        visited.at<uchar>(pt_queue[0]) = 1;
  //
  //        while (head_ptr < queue_len) {
  //          cv::Point &p = pt_queue[head_ptr];
  //          head_ptr++;
  //
  //          if (p.x > 0 && p.x < w - 1 && p.y > 0 && p.y < h - 1) {
  //            int u, v;
  //
  //            u = p.x - 1, v = p.y - 1;
  //            if (local_disp_map.at<uchar>(v, u) > 0 &&
  //                visited.at<uchar>(v, u) == 0) {
  //              pt_queue[queue_len] = cv::Point(u, v);
  //              visited.at<uchar>(v, u) = 1;
  //              queue_len++;
  //            }
  //
  //            u = p.x, v = p.y - 1;
  //            if (local_disp_map.at<uchar>(v, u) > 0 &&
  //                visited.at<uchar>(v, u) == 0) {
  //              pt_queue[queue_len] = cv::Point(u, v);
  //              visited.at<uchar>(v, u) = 1;
  //              queue_len++;
  //            }
  //
  //            u = p.x + 1, v = p.y - 1;
  //            if (local_disp_map.at<uchar>(v, u) > 0 &&
  //                visited.at<uchar>(v, u) == 0) {
  //              pt_queue[queue_len] = cv::Point(u, v);
  //              visited.at<uchar>(v, u) = 1;
  //              queue_len++;
  //            }
  //
  //            u = p.x - 1, v = p.y;
  //            if (local_disp_map.at<uchar>(v, u) > 0 &&
  //                visited.at<uchar>(v, u) == 0) {
  //              pt_queue[queue_len] = cv::Point(u, v);
  //              visited.at<uchar>(v, u) = 1;
  //              queue_len++;
  //            }
  //
  //            u = p.x + 1, v = p.y;
  //            if (local_disp_map.at<uchar>(v, u) > 0 &&
  //                visited.at<uchar>(v, u) == 0) {
  //              pt_queue[queue_len] = cv::Point(u, v);
  //              visited.at<uchar>(v, u) = 1;
  //              queue_len++;
  //            }
  //
  //            u = p.x - 1, v = p.y + 1;
  //            if (local_disp_map.at<uchar>(v, u) > 0 &&
  //                visited.at<uchar>(v, u) == 0) {
  //              pt_queue[queue_len] = cv::Point(u, v);
  //              visited.at<uchar>(v, u) = 1;
  //              queue_len++;
  //            }
  //
  //            u = p.x, v = p.y + 1;
  //            if (local_disp_map.at<uchar>(v, u) > 0 &&
  //                visited.at<uchar>(v, u) == 0) {
  //              pt_queue[queue_len] = cv::Point(u, v);
  //              visited.at<uchar>(v, u) = 1;
  //              queue_len++;
  //            }
  //
  //            u = p.x + 1, v = p.y + 1;
  //            if (local_disp_map.at<uchar>(v, u) > 0 &&
  //                visited.at<uchar>(v, u) == 0) {
  //              pt_queue[queue_len] = cv::Point(u, v);
  //              visited.at<uchar>(v, u) = 1;
  //              queue_len++;
  //            }
  //          }
  //        }
  //
  //        if (queue_len > 500) {
  //          cv::Mat M(queue_len, 3, CV_64FC1),
  //              b(queue_len, 1, CV_64FC1, cv::Scalar(-1));
  //
  //          for (int k = 0; k < queue_len; k++) {
  //            cv::Vec3f &pt = pt_cloud.at<cv::Vec3f>(pt_queue[k]);
  //            M.at<double>(k, 0) = pt[0];
  //            M.at<double>(k, 1) = pt[1];
  //            M.at<double>(k, 2) = pt[2];
  //          }
  //
  //          cv::Mat x = M.inv(cv::DECOMP_SVD) * b;  // x.dot(p)+1=0 is the
  //          plane
  //          cv::Mat n = x / norm(x);                // plane normal
  //          double d = 1.0 / norm(x);  // plane becomes n.dot(p) + d = 0;
  //
  //          cv::Point3d up_dir(0, -1, 0);
  //          cv::Point3d current_plane_normal(n);
  //          double angle_to_up = acos(up_dir.dot(current_plane_normal));
  //
  //          // far from vertical direction
  //          if (angle_to_up > angle_thresh && angle_to_up < PI - angle_thresh)
  //          {
  //            cv::Vec3b c(std::rand() % 200 + 55, std::rand() % 200 + 55,
  //                        std::rand() % 200 + 55);
  //            for (int k = 0; k < queue_len; k++) {
  //              color_labeling.at<cv::Vec3b>(pt_queue[k]) = c;
  //            }
  //          }
  //
  //          //          // draw top view
  //          //          cv::Mat vl = (cv::Mat_<double>(4, 1) << lm.x, lm.y, d,
  //          //          1);
  //          //          cv::Mat vr = (cv::Mat_<double>(4, 1) << rm.x, rm.y, d,
  //          //          1);
  //          //          cv::Mat vb = (cv::Mat_<double>(4, 1) << bm.x, bm.y, d,
  //          //          1);
  //          //          cv::Mat p3l = Q * vl, p3r = Q * vr, p3b = Q * vb;
  //          //          p3l /= p3l.at<double>(3);
  //          //          p3r /= p3r.at<double>(3);
  //          //          p3b /= p3b.at<double>(3);
  //          //
  //          //          if (p3b.at<double>(1) > -1) {
  //          //            double lx = (p3l.at<double>(0) - top_x) * ppm,
  //          //                   ly = (top_z - p3l.at<double>(2)) * ppm;
  //          //            double rx = (p3r.at<double>(0) - top_x) * ppm,
  //          //                   ry = (top_z - p3r.at<double>(2)) * ppm;
  //          //
  //          //            //          cout << lm.x << ", " << lm.y << "===="
  //          <<
  //          //            lx << ", " <<
  //          //            //          ly
  //          //            //          << endl;
  //          //            //          cout << rm.x << ", " << rm.y << "===="
  //          <<
  //          //            rx << ", " <<
  //          //            //          ry
  //          //            //          << endl
  //          //            //               << endl;
  //          //
  //          //            cv::line(topview, cv::Point(lx, ly), cv::Point(rx,
  //          //            ry),
  //          //                     cv::Scalar(255, 255, 255), 2);
  //          //          }
  //        }
  //      }
  //    }
  //  }

  //    vector<uchar> maj_disp(v_disp.rows, 255);
  //    for (int i = v_disp.rows - 1; i > v_disp.rows / 2; i--) {
  //      double maxv;
  //      cv::Point max_loc;
  //      cv::minMaxLoc(v_disp.row(i), NULL, &maxv, NULL, &max_loc);
  //      int d = max_loc.x;
  //      if (maxv > disp_map.cols / 20.0) {
  //        maj_disp[i] = d;
  //      }
  //    }

  // fit planes to rest 3D points
  //  cv::Mat pt_cloud;
  //  reprojectImageTo3D(local_disp_map, pt_cloud, Q, true);
  //
  //  vector<cv::Point3f> pts;
  //  vector<cv::Point> coords;
  //  for (int i = 0; i < local_disp_map.rows; i++) {
  //    for (int j = 0; j < local_disp_map.cols; j++) {
  //      cv::Vec3f &pt = pt_cloud.at<cv::Vec3f>(i, j);
  //      //      if (local_disp_map.at<uchar>(i, j) > 0 &&
  //      //          local_disp_map.at<uchar>(i, j) != maj_disp[i] && pt[1] >
  //      // 1.f
  //      //          &&
  //      //          pt[1] < 2.f)
  //      if (local_disp_map.at<uchar>(i, j) > 0 && pt[1] > 0) {
  //        pts.push_back(cv::Point3f(pt));
  //        coords.push_back(cv::Point(j, i));
  //      } else
  //        color_labeling.at<cv::Vec3b>(i, j) = cv::Vec3b(0, 0, 0);
  //    }
  //  }
  //
  //  int num_inliers;
  //  const int NUM_THRESH = 1000;
  //  do {
  //    cv::Vec4d plane;
  //    vector<uchar> in_flags;
  //    ransacPlaneFitting(pts, 100, 80.f / 180.f * PI, 0.2f, plane, in_flags,
  //                       num_inliers);
  //    if (num_inliers > NUM_THRESH) {
  //      vector<cv::Point3f> remaining_pts;
  //      vector<cv::Point> remaining_coords;
  //
  //      cv::Vec3b c(std::rand() % 200 + 55, std::rand() % 200 + 55,
  //                  std::rand() % 200 + 55);
  //      for (int i = 0; i < in_flags.size(); i++) {
  //        if (in_flags[i])
  //          color_labeling.at<cv::Vec3b>(coords[i].y, coords[i].x) = c;
  //        else {
  //          remaining_pts.push_back(pts[i]);
  //          remaining_coords.push_back(coords[i]);
  //        }
  //      }
  //      pts = remaining_pts;
  //      coords = remaining_coords;
  //      cout << "plane found: " << num_inliers << endl;
  //    }
  //  } while (num_inliers > NUM_THRESH);
}

void verticalPlaneFinder(cv::Mat &disp_map, cv::Mat &Q) {
  cv::Mat local_disp_map = disp_map.clone();
  cv::Mat pt_cloud;
  reprojectImageTo3D(local_disp_map, pt_cloud, Q, true);

  float ground_height = 0.4f, tol_height = 2.f;
  float h_lower = ground_height - tol_height,
        h_upper = ground_height + tol_height;

  // filter out 3D points that are high and low and far
  for (int i = 0; i < pt_cloud.rows; i++) {
    for (int j = 0; j < pt_cloud.cols; j++) {
      cv::Vec3f &pt = pt_cloud.at<cv::Vec3f>(i, j);
      if (pt[1] < h_lower || pt[1] > h_upper || pt[2] >= 10000) {
        local_disp_map.at<uchar>(i, j) = 0;
      }
    }
  }

  cv::Mat color_disp_map_samed, color_disp_map_vplanes;
  cv::cvtColor(local_disp_map, color_disp_map_samed, CV_GRAY2RGB);
  cv::cvtColor(local_disp_map, color_disp_map_vplanes, CV_GRAY2RGB);

  std::map<uchar, int> disp_cnt;
  for (int i = 0; i < local_disp_map.rows; i++) {
    for (int j = 0; j < local_disp_map.cols; j++) {
      uchar dp = local_disp_map.at<uchar>(i, j);
      disp_cnt[dp]++;
    }
  }

  const int OBJ_PIXEL_THRESH = 1000;
  for (std::map<uchar, int>::iterator it = disp_cnt.begin();
       it != disp_cnt.end(); it++) {
    cout << it->second << endl;
    if (it->second > OBJ_PIXEL_THRESH && it->first > 0) {
      cv::Vec3b c(std::rand() % 200 + 55, std::rand() % 200 + 55,
                  std::rand() % 200 + 55);
      cv::Mat mask = local_disp_map == it->first;
      color_disp_map_samed.setTo(c, mask);
      local_disp_map.setTo(cv::Scalar(0), mask);
    }
  }

  vector<cv::Point3f> pts;
  vector<cv::Point> coords;
  for (int i = 0; i < local_disp_map.rows; i++) {
    for (int j = 0; j < local_disp_map.cols; j++) {
      if (local_disp_map.at<uchar>(i, j) > 0) {
        cv::Vec3f &pt = pt_cloud.at<cv::Vec3f>(i, j);
        pts.push_back(cv::Point3f(pt));
        coords.push_back(cv::Point(j, i));
      }
    }
  }

  int num_inliers;
  const int NUM_THRESH = 1000;
  do {
    cv::Vec4d plane;
    vector<uchar> in_flags;
    ransacPlaneFitting(pts, 500, 45.f / 180.f * PI, 0.2f, plane, in_flags,
                       num_inliers);
    if (num_inliers > NUM_THRESH) {
      vector<cv::Point3f> remaining_pts;
      vector<cv::Point> remaining_coords;

      cv::Vec3b c(std::rand() % 200 + 55, std::rand() % 200 + 55,
                  std::rand() % 200 + 55);
      for (int i = 0; i < in_flags.size(); i++) {
        if (in_flags[i])
          color_disp_map_vplanes.at<cv::Vec3b>(coords[i].y, coords[i].x) = c;
        else {
          remaining_pts.push_back(pts[i]);
          remaining_coords.push_back(coords[i]);
        }
      }
      pts = remaining_pts;
      coords = remaining_coords;
      cout << "plane found: " << num_inliers << endl;
    }
  } while (num_inliers > NUM_THRESH);

  imwrite("disp_samed.png", color_disp_map_samed);
  imwrite("disp_vplane.png", color_disp_map_vplanes);
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

  cv::Mat left_gray, right_gray, left_color, right_color;
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
  fs["R1"] >> R1;
  fs["R2"] >> R2;

  t = t / norm(t) * baseline;

  //  cv::Mat R_tmp, rvec(3, 1, CV_64FC1, cv::Scalar(0));
  //  rvec.at<double>(1) = -3.14 / 300;
  //  cv::Rodrigues(rvec, R_tmp);
  //  R = R_tmp * R;

  //  cv::Rect roi(0, img_h / 3, img_w, img_h / 3);
  //  int s = 1;
  cv::Rect roi(0, 0, img_w, img_h);
  int s = 2;

  //  cv::stereoRectify(K_left, dist_coeffs_left, K_right, dist_coeffs_right,
  //                    cv::Size(img_w, img_h), R, t, R1, R2, P1, P2, Q,
  //                    CV_CALIB_ZERO_DISPARITY, 0, cv::Size(img_w / s, img_h
  //                    /
  //                    s),
  //                    &validRoi[0], &validRoi[1]);
  //  img_w = img_w / s;
  //  img_h = img_h / s;
  //  cv::Mat shift = cv::Mat::eye(3, 3, CV_64FC1);
  //  shift.at<double>(0, 2) = 0.044;
  //  R2 = shift * R2;
  //  cout << "Q:" << endl << Q << endl;

  cv::Mat newK = K_left.clone();

  // roi
  newK.at<double>(0, 2) -= roi.x;
  newK.at<double>(1, 2) -= roi.y;

  // scaling
  newK.at<double>(0, 0) /= s;
  newK.at<double>(0, 2) /= s;
  newK.at<double>(1, 1) /= s;
  newK.at<double>(1, 2) /= s;
  int new_imgw = roi.width / s, new_imgh = roi.height / s;

  cv::Mat left_map[2], right_map[2];
  initUndistortRectifyMap(K_left, dist_coeffs_left, R1, newK,
                          cv::Size(new_imgw, new_imgh), CV_16SC2, left_map[0],
                          left_map[1]);
  initUndistortRectifyMap(K_right, dist_coeffs_right, R2, newK,
                          cv::Size(new_imgw, new_imgh), CV_16SC2, right_map[0],
                          right_map[1]);

  Q = cv::Mat(4, 4, CV_64FC1, cv::Scalar(0));
  Q.at<double>(0, 0) = 1;
  Q.at<double>(0, 3) = -newK.at<double>(0, 2);
  Q.at<double>(1, 1) = 1;
  Q.at<double>(1, 3) = -newK.at<double>(1, 2);
  Q.at<double>(2, 3) = newK.at<double>(0, 0);
  Q.at<double>(3, 2) = 1 / baseline;

  cout << Q << endl;

  cudaGLSetGLDevice(0);

  SGMDemo demo(new_imgw, new_imgh);
  if (demo.init()) {
    printf("fail to init SGM Demo\n");
    std::exit(EXIT_FAILURE);
  }

  sgm::StereoSGM ssgm(new_imgw, new_imgh, disp_size, 8, 16,
                      sgm::EXECUTE_INOUT_HOST2CUDA);

  Renderer renderer(new_imgw, new_imgh);

  uint16_t *d_output_buffer = NULL;

  cv::namedWindow("stereo", cv::WINDOW_NORMAL);
  cv::namedWindow("left_gray_rectified", cv::WINDOW_NORMAL);
  cv::namedWindow("right_gray_rectified", cv::WINDOW_NORMAL);
  cv::Mat color_depth_map(new_imgh, new_imgw, CV_8UC3);
  // int frame_no = 0;
  running.store(true);
  bool continuous_saving = false;
  int out_cnt = 0;
  cv::Mat stereo_color;
  bool pause_it = false;
  while (running.load()) {
    glClearColor(0.0, 0.0, 0.0, 1.0);
    glClear(GL_COLOR_BUFFER_BIT);

    //    double left_ts, right_ts;
    //    cv::Mat left_color = getImageFromBF(left_subscriber, left_ts);
    //    cv::Mat right_color = getImageFromBF(right_subscriber, right_ts);

    double stereo_ts;
    timespec tp;
    if (!pause_it)
      stereo_color = getImageFromBF(stereo_subscriber, stereo_ts, tp);

    if (!stereo_color.empty()) {
      //////////////////////////////////////////////////////////////////////////////////
      //      cv::Mat left_orig = stereo_color(
      //                  cv::Rect(0, 0, stereo_color.cols / 2,
      //                  stereo_color.rows)),
      //              right_orig = stereo_color(cv::Rect(stereo_color.cols /
      //              2,
      //              0,
      //                                                 stereo_color.cols /
      //                                                 2,
      //                                                 stereo_color.rows));
      //
      //      cv::Mat left_center =
      //                  left_orig(cv::Rect(left_orig.cols / 4,
      //                  left_orig.rows
      //                  / 4,
      //                                     left_orig.cols / 2,
      //                                     left_orig.rows
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

      left_color = stereo_color(
          cv::Rect(0, 0, stereo_color.cols / 2, stereo_color.rows));
      right_color = stereo_color(cv::Rect(
          stereo_color.cols / 2, 0, stereo_color.cols / 2, stereo_color.rows));

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
      full_size_disp.convertTo(disp_8u, CV_8U);
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

      //      // compute v-disparity
      //      cv::Mat v_disp, v_disp_float, v_disp_mask, vdisp_show;
      //      computeVDisparity(disp_8u, v_disp);
      //      // v_disp.convertTo(v_disp_show, CV_8U);
      //      v_disp.convertTo(v_disp_float, CV_32F);
      //      cv::threshold(v_disp_float, v_disp_mask, disp_8u.cols / 5, 255,
      //                    cv::THRESH_BINARY);
      //      v_disp_mask.convertTo(v_disp_mask, CV_8U);
      //      cv::namedWindow("v_disp", cv::WINDOW_FREERATIO);
      //      cv::cvtColor(v_disp_mask, vdisp_show, CV_GRAY2RGB);
      //      imshow("v_disp", vdisp_show);

      cv::Mat cc_label, topview;
      connectedComponentsVPlane(disp_8u, cc_label, Q, topview);
      cv::namedWindow("cc_labels", cv::WINDOW_NORMAL);
      imshow("cc_labels", cc_label);
      cv::namedWindow("topview", cv::WINDOW_NORMAL);
      imshow("topview", topview);

      // imshow("v_disp", v_disp_show);

      // vector<cv::Point> locations;  // output, locations of non-zero pixels
      // cv::findNonZero(v_disp_mask, locations);
      //      if (locations.size() > 100) {
      //        cv::Vec4f aline;
      //        cv::fitLine(locations, aline, CV_DIST_L12, 0, 0.01, 0.01);
      //        cv::Point2f line_p(aline[2], aline[3]), line_v(aline[0],
      //        aline[1]);
      //        cv::Point2f draw_line_p1 = line_p + 10000.f * line_v,
      //                    draw_line_p2 = line_p - 10000.f * line_v;
      //        cv::line(vdisp_show, cv::Point(draw_line_p1),
      //        cv::Point(draw_line_p2),
      //                 cv::Scalar(0, 255, 0));
      //      }

      //      vector<cv::Vec2f> lines;
      //      cv::HoughLines(v_disp_mask, lines, 3, CV_PI / 180 * 10, 50, 0,
      //      0);
      //      for (int i = 0; i < lines.size(); i++) {
      //        float rho = lines[i][0], theta = lines[i][1];
      //        cv::Point pt1, pt2;
      //        double a = cos(theta), b = sin(theta);
      //        double x0 = a * rho, y0 = b * rho;
      //        pt1.x = cvRound(x0 + 1000 * (-b));
      //        pt1.y = cvRound(y0 + 1000 * (a));
      //        pt2.x = cvRound(x0 - 1000 * (-b));
      //        pt2.y = cvRound(y0 - 1000 * (a));
      //
      //        cv::Vec3b c(std::rand() % 200 + 55, std::rand() % 200 + 55,
      //                    std::rand() % 200 + 55);
      //        cv::line(vdisp_show, pt1, pt2, c);
      //      }

      // show line fitting of v-disparity
      //      vector<cv::Vec4f> lines;
      //      cv::Mat tmp1(vdisp_show.size(), CV_8UC3, cv::Scalar(0, 0, 0));
      //      HoughLinesP(v_disp_mask, lines, 1, CV_PI / 180, v_disp_mask.rows
      //      /
      //      10,
      //                  v_disp_mask.rows / 10, v_disp_mask.rows);
      //      float max_line_len = 0;
      //      cv::Vec4f g_line;
      //      for (size_t i = 0; i < lines.size(); i++) {
      //        cv::Vec3b c(std::rand() % 200 + 55, std::rand() % 200 + 55,
      //                    std::rand() % 200 + 55);
      //        cv::line(tmp1, cv::Point(lines[i][0], lines[i][1]),
      //                 cv::Point(lines[i][2], lines[i][3]), c, 1, 8);
      //
      //        cv::Point2f p1(lines[i][0], lines[i][1]), p2(lines[i][2],
      //        lines[i][3]);
      //        cv::Point2f line_v = p2 - p1;
      //        float angle = std::atan2(line_v.y, line_v.x) * 180 / CV_PI;
      //        float line_len = norm(p2 - p1);
      //        if (fabs(angle - 75) < 5 && line_len > max_line_len) {
      //          max_line_len = line_len;
      //
      //          g_line = lines[i];
      //        }
      //      }
      //
      //      cv::namedWindow("tmp1", cv::WINDOW_FREERATIO);
      //      imshow("tmp1", tmp1);

      //      cv::Mat ground_label = left_color_rectified.clone();
      //      for (int i = v_disp.rows - 1; i > v_disp.rows / 2; i--) {
      //        double maxv;
      //        cv::Point max_loc;
      //        cv::minMaxLoc(v_disp.row(i), NULL, &maxv, NULL, &max_loc);
      //        int d = max_loc.x;
      //        for (int j = 0; j < disp_8u.cols; j++) {
      //          if (abs(disp_8u.at<uchar>(i, j) - d) < 2) {
      //            ground_label.at<cv::Vec3b>(i, j) += cv::Vec3b(0, 0, 100);
      //          }
      //        }
      //      }

      // disparity with ground removed
      //      cv::Mat ground_label;
      //      cv::flip(color_depth_map, ground_label, 0);
      //      if (max_line_len > 0) {
      //        cv::Point2f p1(g_line[0], g_line[1]), p2(g_line[2],
      //        g_line[3]);
      //        cv::Point2f line_v = p2 - p1;
      //        line_v /= norm(line_v);
      //        for (int i = 0; i < ground_label.rows; i++) {
      //          float delta = (i - p1.y) / line_v.y;
      //          int d = (int)(delta * line_v.x + p1.x + 0.5);
      //          if (d > 5 && d < 128) {
      //            for (int j = 0; j < disp_8u.cols; j++) {
      //              if (abs((int)disp_8u.at<uchar>(i, j) - d) < 2) {
      //                ground_label.at<cv::Vec3b>(i, j) = cv::Vec3b(255, 0,
      //                0);
      //              }
      //            }
      //          }
      //        }
      //      }
      //
      //      cv::imshow("ground_label", ground_label);

      char c = cv::waitKey(10);

      if (c == 'd') pause_it = !pause_it;

      // save intermediate results
      if (c == 's') {
        cv::Mat ds_disp;

        double ds = 0.5;

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

        cv::imwrite("disp.jpg", disp_8u);

        cv::Mat u_disp(128, disp_8u.cols, CV_32SC1, cv::Scalar(0));

        for (int i = 0; i < disp_8u.rows; i++) {
          for (int j = 0; j < disp_8u.cols; j++) {
            uchar d = disp_8u.at<uchar>(i, j);
            if (d > 0 && d < 128) {
              u_disp.at<int>(d, j)++;
            }
          }
        }

        cv::Mat u_disp_vis;
        u_disp.convertTo(u_disp_vis, CV_8U, 2);
        imwrite("u_disp.png", u_disp_vis);

        imwrite("topview.png", topview);

        // verticalPlaneFinder(disp_8u, Q);
      }

      if (c == 'c') continuous_saving = !continuous_saving;

      // save intermediate results
      if (continuous_saving) {
        char name[256];
        sprintf(name, "color%04d.png", out_cnt);
        imwrite(name, left_color_rectified);
        sprintf(name, "disp%04d.png", out_cnt);
        imwrite(name, disp_8u);
        sprintf(name, "orig_right%04d.png", out_cnt);
        imwrite(name, right_color);

        out_cnt++;

        cv::FileStorage fs("Q.xml", cv::FileStorage::WRITE);
        fs << "Q" << Q;

        cout << out_cnt << endl;
      }
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
}
