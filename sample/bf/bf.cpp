/*
 * ----------------------------------------------------------------------------
 * Binary Flow (bf): A Minimalist IPC middleware
 * Copyright (C) 2016 AutoX, Inc.
 * ----------------------------------------------------------------------------
 */

#include "bf.hpp"

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <signal.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/types.h>
#include <time.h>
#include <unistd.h>
#include <chrono>
#include <future>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>
#ifdef __MACH__
#include <mach/clock.h>
#include <mach/mach.h>
#endif
#include <errno.h>
#include <fcntl.h>
#include <fstream>
#include <iostream>
#include <sstream>

#define kTIMEOUT_S_NONBLOCK 0
#define kTIMEOUT_MS_NONBLOCK 500
#define kNONBLOCK_RETRY_CONNECT_COUNT 10
#define kPUBLISHER_DATA_SERVER_CONNECTION_RETRY_COUNT \
  10  // 10 * 500ms = 5000ms = 5s
#define kSLEEP_AFTER_NONBLOCK_FAIL_S 6
#define kPUBLISHER_TIME_TO_SLEEP_BETWEEN_LISTEN_READY_MS 10

#define kUSE_SHM 1
#define kSHM_PREFIX "/tmp/"

namespace bf {

bool get_time(timespec* ptms) {
#ifdef __MACH__  // OS X does not have clock_get_time, use clock_get_time
  clock_serv_t cclock;
  mach_timespec_t mts;
  host_get_clock_service(mach_host_self(), CALENDAR_CLOCK, &cclock);
  clock_get_time(cclock, &mts);
  mach_port_deallocate(mach_task_self(), cclock);
  ptms->tv_sec = mts.tv_sec;
  ptms->tv_nsec = mts.tv_nsec;
  return false;
#else
  return clock_gettime(CLOCK_REALTIME, ptms);
#endif
}

void print_time(timespec t) {
  std::cout << t.tv_sec << "." << std::setfill('0') << std::setw(9)
            << t.tv_nsec;
}

timespec subtract_time(timespec a, timespec b) {
  timespec d;
  d.tv_sec = a.tv_sec - b.tv_sec;
  if (a.tv_nsec >= b.tv_nsec) {
    d.tv_nsec = a.tv_nsec - b.tv_nsec;
  } else {
    d.tv_sec = d.tv_sec - 1;
    d.tv_nsec = 1000000000 + a.tv_nsec - b.tv_nsec;
  }
  return d;
}

bool a_eq_later_than_b(timespec a, timespec b) {
  if (a.tv_sec > b.tv_sec)
    return true;
  else if (a.tv_sec < b.tv_sec)
    return false;
  else
    return (a.tv_nsec >= b.tv_nsec);
}

bool a_same_time_with_b(timespec a, timespec b) {
  return (a.tv_sec == b.tv_sec) && (a.tv_nsec == b.tv_nsec);
}

Client::Client(std::string host_name, unsigned short host_port, bool blocking,
               bool* pResult) {
  struct sockaddr_in serv_addr;
  struct hostent* server;
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0) {
    std::cout << "ERROR " << strerror(errno) << ", opening socket at "
              << __FILE__ << ":" << __LINE__ << std::endl;
    exit(0);
  }

  if (blocking && (pResult == NULL)) {
    std::cout << "ERROR cannot have a blocking client and no pResult "
              << std::endl;
    exit(0);
  }

  blocking_client = blocking;
  if (!blocking_client) {
    if (fcntl(sockfd, F_SETFL, O_NONBLOCK) < 0) {
      std::cout << "ERROR " << strerror(errno)
                << ", setting socket to nonblock at " << __FILE__ << ":"
                << __LINE__ << std::endl;
      exit(0);
    }
  }

  bzero((char*)&serv_addr, sizeof(serv_addr));
  server = gethostbyname(host_name.c_str());
  serv_addr.sin_family = AF_INET;
  if (server == NULL) {
    std::cout << "Cannot find such host, trying to connect via IP";
    serv_addr.sin_addr.s_addr = inet_addr(host_name.c_str());
  } else {
    bcopy((char*)server->h_addr, (char*)&serv_addr.sin_addr.s_addr,
          server->h_length);
  }
  serv_addr.sin_port = htons(host_port);

  bool retVal = true;

  int result = 0;

  result = connect(sockfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr));
  if (result < 0) {
    if (blocking_client) {
      retVal = false;
      static int notification_counter = kNotificationWaitTime;
      notification_counter++;
      if (notification_counter > kNotificationWaitTime) {
        notification_counter = 0;
        std::cout << "ERROR " << strerror(errno) << ", connecting at "
                  << __FILE__ << ":" << __LINE__ << std::endl;
      }
    } else {
      retVal = false;

      if (errno == EINPROGRESS) {
        fd_set temp_working_list;
        fd_set temp_master_list;
        FD_ZERO(&temp_working_list);
        FD_ZERO(&temp_master_list);
        FD_SET(sockfd, &temp_master_list);

        temp_working_list = temp_master_list;
        result = select(sockfd + 1, NULL, &temp_working_list, NULL, NULL);

        if (result < 0) {
          std::cout << "ERROR " << strerror(errno) << ", on select at "
                    << __FILE__ << ":" << __LINE__ << std::endl;
          goto after_select;
        }

        socklen_t result_len = sizeof(result);
        if (getsockopt(sockfd, SOL_SOCKET, SO_ERROR, &result, &result_len) <
            0) {
          std::cout << "ERROR " << strerror(errno)
                    << ", connecting to NONBLOCK socket at " << __FILE__ << ":"
                    << __LINE__ << std::endl;
          goto after_select;
        }

        if (result == 0) {
          // connected succesfully -- retVal = true;
          // std::cout << "SOERROR = SUCCESS \n";
          retVal = true;
        } else {
          std::cout << "ERROR [" << strerror(result)
                    << "] connecting to NONBLOCK socket at " << __FILE__ << ":"
                    << __LINE__ << std::endl;
          auto now = std::chrono::system_clock::now();
          auto now_c = std::chrono::system_clock::to_time_t(now);
          auto seconds =
              std::chrono::time_point_cast<std::chrono::seconds>(now);
          auto fraction = now - seconds;
          auto milliseconds =
              std::chrono::duration_cast<std::chrono::milliseconds>(fraction);

          std::cout << "BF FATAL FAILURE NONBLOCK CLIENT @ "
                    << std::put_time(std::localtime(&now_c), "%c")
                    << std::to_string(milliseconds.count()) << "\n";
        }
      }  // errno == EIN_PROGRESS

    }  // if blocking_client
  }    // if connect result < 0

after_select:
  if (pResult != NULL) *pResult = retVal;

  if (!retVal) {
    if (pResult) {
      return;
    }

    std::cout << "FAILURE CONNECTING TO " << std::to_string(host_port)
              << std::endl;
    std::cout << "PLEASE DO NOT PRESS CTRL+C AND WAIT 5 SECONDS FOR THE "
                 "BINARY TO EXIT GRACEFULLY!!"
              << std::endl;
    std::cout << "YOU CAN RELAUNCH AFTER THE BINARY EXITS. THANK YOU"
              << std::endl;
    usleep(kSLEEP_AFTER_NONBLOCK_FAIL_S * 1000000);
    exit(0);
  }

  // successful here
  if (!blocking_client) {
    FD_ZERO(&master_list);
    FD_ZERO(&working_list);
    FD_SET(sockfd, &master_list);
  }

  client_active.store(true);
}

int Client::read(void** pbuffer, timespec* ptms, char* data_type,
                 unsigned long* matrix_width, unsigned long* matrix_height,
                 unsigned int* matrix_type) {
  size_t size_of_msg = 0;

  int n;
  bool free_ptms = false;
  if (ptms == NULL) {
    free_ptms = true;
    ptms = (timespec*)malloc(sizeof(struct timespec));
  }

  timeval select_timeout;
  select_timeout.tv_sec = kTIMEOUT_S_NONBLOCK;
  select_timeout.tv_usec = kTIMEOUT_MS_NONBLOCK * 1000;  // 500ms

  int size_to_read = 0;
  int size_read = 0;

  bool free_data_type = (data_type == NULL);
  if (free_data_type) {
    data_type = (char*)malloc(sizeof(char));
  }

  size_to_read = sizeof(*data_type);
  size_read = 0;

  while (size_read < size_to_read) {
    if (!blocking_client) {
      while (client_active.load()) {
        working_list = master_list;
        select_timeout.tv_sec = kTIMEOUT_S_NONBLOCK;
        select_timeout.tv_usec = kTIMEOUT_MS_NONBLOCK * 1000;

        n = select(sockfd + 1, &working_list, NULL, NULL, &select_timeout);

        if (n == -1) {
          std::cout << "ERROR " << strerror(errno) << ", on select at "
                    << __FILE__ << ":" << __LINE__ << std::endl;
          exit(0);
        }

        if (n == 1) break;
      }
    }

    n = ::read(sockfd, (char*)(data_type) + size_read,
               size_to_read - size_read);

    if (n <= 0) {
      break;
    }

    size_read += n;
  }

  if (n <= 0) {
    if (n == 0) {
      std::cout << "ERROR " << strerror(errno) << ", reading from socket at "
                << __FILE__ << ":" << __LINE__
                << " ZERO BYTE. Read so far: " << size_read << std::endl
                << std::flush;
      n = -1;
    } else {
      std::cout << "ERROR " << strerror(errno) << ", reading from socket at "
                << __FILE__ << ":" << __LINE__ << std::endl;
    }
    return n;
  }

  if (*data_type == kDATA_TYPE_MATRIX) {
    bool free_width = (matrix_width == NULL);
    if (free_width) {
      matrix_width = (unsigned long*)malloc(sizeof(matrix_width));
    }

    // read 3 extra things -- width
    size_to_read = sizeof(*matrix_width);
    size_read = 0;

    while (size_read < size_to_read) {
      if (!blocking_client) {
        while (client_active.load()) {
          working_list = master_list;
          select_timeout.tv_sec = kTIMEOUT_S_NONBLOCK;
          select_timeout.tv_usec = kTIMEOUT_MS_NONBLOCK * 1000;

          n = select(sockfd + 1, &working_list, NULL, NULL, &select_timeout);

          if (n == -1) {
            std::cout << "ERROR " << strerror(errno) << ", on select at "
                      << __FILE__ << ":" << __LINE__ << std::endl;
            exit(0);
          }

          if (n == 1) break;
        }
      }

      n = ::read(sockfd, (char*)(matrix_width) + size_read,
                 size_to_read - size_read);

      if (n <= 0) {
        break;
      }

      size_read += n;
    }

    if (free_width) {
      free(matrix_width);
      matrix_width = NULL;
    }

    if (n <= 0) {
      if (n == 0) {
        std::cout << "ERROR " << strerror(errno) << ", reading from socket at "
                  << __FILE__ << ":" << __LINE__
                  << " ZERO BYTE. Read so far: " << size_read << std::endl
                  << std::flush;
        n = -1;
      } else {
        std::cout << "ERROR " << strerror(errno) << ", reading from socket at "
                  << __FILE__ << ":" << __LINE__ << std::endl;
      }
      return n;
    }

    bool free_height = (matrix_height == NULL);
    if (free_height) {
      matrix_height = (unsigned long*)malloc(sizeof(matrix_height));
    }

    // read 3 extra things -- height
    size_to_read = sizeof(*matrix_height);
    size_read = 0;

    while (size_read < size_to_read) {
      if (!blocking_client) {
        while (client_active.load()) {
          working_list = master_list;
          select_timeout.tv_sec = kTIMEOUT_S_NONBLOCK;
          select_timeout.tv_usec = kTIMEOUT_MS_NONBLOCK * 1000;

          n = select(sockfd + 1, &working_list, NULL, NULL, &select_timeout);

          if (n == -1) {
            std::cout << "ERROR " << strerror(errno) << ", on select at "
                      << __FILE__ << ":" << __LINE__ << std::endl;
            exit(0);
          }

          if (n == 1) break;
        }
      }

      n = ::read(sockfd, (char*)(matrix_height) + size_read,
                 size_to_read - size_read);

      if (n <= 0) {
        break;
      }

      size_read += n;
    }

    if (free_height) {
      free(matrix_height);
      matrix_height = NULL;
    }

    if (n <= 0) {
      if (n == 0) {
        std::cout << "ERROR " << strerror(errno) << ", reading from socket at "
                  << __FILE__ << ":" << __LINE__
                  << " ZERO BYTE. Read so far: " << size_read << std::endl
                  << std::flush;
        n = -1;
      } else {
        std::cout << "ERROR " << strerror(errno) << ", reading from socket at "
                  << __FILE__ << ":" << __LINE__ << std::endl;
      }
      return n;
    }

    bool free_type = (matrix_type == NULL);
    if (free_type) {
      matrix_type = (unsigned int*)malloc(sizeof(matrix_type));
    }

    // read 3 extra things -- type
    size_to_read = sizeof(*matrix_type);
    size_read = 0;

    while (size_read < size_to_read) {
      if (!blocking_client) {
        while (client_active.load()) {
          working_list = master_list;
          select_timeout.tv_sec = kTIMEOUT_S_NONBLOCK;
          select_timeout.tv_usec = kTIMEOUT_MS_NONBLOCK * 1000;

          n = select(sockfd + 1, &working_list, NULL, NULL, &select_timeout);

          if (n == -1) {
            std::cout << "ERROR " << strerror(errno) << ", on select at "
                      << __FILE__ << ":" << __LINE__ << std::endl;
            exit(0);
          }

          if (n == 1) break;
        }
      }

      n = ::read(sockfd, (char*)(matrix_type) + size_read,
                 size_to_read - size_read);

      if (n <= 0) {
        break;
      }

      size_read += n;
    }

    if (free_type) {
      free(matrix_type);
      matrix_type = NULL;
    }

    if (n <= 0) {
      if (n == 0) {
        std::cout << "ERROR " << strerror(errno) << ", reading from socket at "
                  << __FILE__ << ":" << __LINE__
                  << " ZERO BYTE. Read so far: " << size_read << std::endl
                  << std::flush;
        n = -1;
      } else {
        std::cout << "ERROR " << strerror(errno) << ", reading from socket at "
                  << __FILE__ << ":" << __LINE__ << std::endl;
      }
      return n;
    }
  }

  if (free_data_type) {
    free(data_type);
    data_type = NULL;
  }

  size_to_read = sizeof(struct timespec);
  size_read = 0;

  while (size_read < size_to_read) {
    if (!blocking_client) {
      while (client_active.load()) {
        working_list = master_list;
        select_timeout.tv_sec = kTIMEOUT_S_NONBLOCK;
        select_timeout.tv_usec = kTIMEOUT_MS_NONBLOCK * 1000;

        n = select(sockfd + 1, &working_list, NULL, NULL, &select_timeout);

        if (n == -1) {
          std::cout << "ERROR " << strerror(errno) << ", on select at "
                    << __FILE__ << ":" << __LINE__ << std::endl;
          exit(0);
        }

        if (n == 1) break;
      }
    }

    n = ::read(sockfd, (char*)(ptms) + size_read, size_to_read - size_read);

    if (n <= 0) {
      break;
    }

    size_read += n;
  }

  if (free_ptms) {
    free(ptms);
    ptms = NULL;
  }

  if (n <= 0) {
    if (n == 0) {
      std::cout << "ERROR " << strerror(errno) << ", reading from socket at "
                << __FILE__ << ":" << __LINE__
                << " ZERO BYTE. Read so far: " << size_read
                << " size to read: " << size_to_read << std::endl
                << std::flush;
      n = -1;
    } else {
      std::cout << "ERROR " << strerror(errno) << ", reading from socket at "
                << __FILE__ << ":" << __LINE__ << std::endl;
    }
    return n;
  }
  // std::cout << "read ptms " << std::endl;

  size_to_read = sizeof(size_of_msg);
  size_read = 0;

  while (size_read < size_to_read) {
    if (!blocking_client) {
      while (client_active.load()) {
        working_list = master_list;
        select_timeout.tv_sec = kTIMEOUT_S_NONBLOCK;
        select_timeout.tv_usec = kTIMEOUT_MS_NONBLOCK * 1000;

        n = select(sockfd + 1, &working_list, NULL, NULL, &select_timeout);

        if (n == -1) {
          std::cout << "ERROR " << strerror(errno) << ", on select at "
                    << __FILE__ << ":" << __LINE__ << std::endl;
          exit(0);
        }

        if (n == 1) break;
      }
    }

    n = ::read(sockfd, (char*)(&size_of_msg) + size_read,
               size_to_read - size_read);

    if (n <= 0) {
      break;
    }

    size_read += n;
  }

  if (n <= 0) {
    if (n == 0) {
      std::cout << "ERROR " << strerror(errno) << ", reading from socket at "
                << __FILE__ << ":" << __LINE__
                << " ZERO BYTE. Read so far: " << size_read
                << " size to read: " << size_to_read << std::endl
                << std::flush;
      n = -1;
    } else {
      std::cout << "ERROR " << strerror(errno) << ", reading from socket at "
                << __FILE__ << ":" << __LINE__ << std::endl;
    }
    return n;
  }

  if (size_of_msg == 0) return 0;

  // std::cout << "read size: " << size_of_msg << std::endl;

  if (*pbuffer == NULL) *pbuffer = malloc(size_of_msg);

  size_to_read = size_of_msg;
  size_read = 0;

  while (size_read < size_to_read) {
    if (!blocking_client) {
      while (client_active.load()) {
        working_list = master_list;
        select_timeout.tv_sec = kTIMEOUT_S_NONBLOCK;
        select_timeout.tv_usec = kTIMEOUT_MS_NONBLOCK * 1000;

        n = select(sockfd + 1, &working_list, NULL, NULL, &select_timeout);

        if (n == -1) {
          std::cout << "ERROR " << strerror(errno) << ", on select at "
                    << __FILE__ << ":" << __LINE__ << std::endl;
          exit(0);
        }

        if (n == 1) break;
      }
    }

    n = ::read(sockfd, (char*)(*pbuffer) + size_read, size_to_read - size_read);

    if (n <= 0) {
      break;
    }

    size_read += n;
  }

  if (n <= 0) {
    if (n == 0) {
      std::cout << "ERROR " << strerror(errno) << ", reading from socket at "
                << __FILE__ << ":" << __LINE__
                << " ZERO BYTE. Read so far: " << size_read
                << " size to read: " << size_to_read << std::endl
                << std::flush;
      n = -1;
    } else {
      std::cout << "ERROR " << strerror(errno) << ", reading from socket at "
                << __FILE__ << ":" << __LINE__ << std::endl;
    }
    return n;
  }

  return size_of_msg;
}

int Client::write(const void* buffer, size_t size, timespec* ptms) {
  int n;
  bool free_ptms = false;
  if (ptms == NULL) {
    free_ptms = true;
    ptms = (timespec*)malloc(sizeof(timespec));
    if (get_time(ptms)) {
      std::cout << "ERROR get_time";
      exit(0);
    }
  }

  timeval select_timeout;
  select_timeout.tv_sec = kTIMEOUT_S_NONBLOCK;
  select_timeout.tv_usec = kTIMEOUT_MS_NONBLOCK * 1000;  // 500ms

  int size_to_write = 0;
  int size_written = 0;
  char data_type = kDATA_TYPE_DATA;

  // first send the data type (char)
  size_to_write = sizeof(data_type);
  size_written = 0;

  while (size_written < size_to_write) {
    if (!blocking_client) {
      while (client_active.load()) {
        working_list = master_list;
        select_timeout.tv_sec = kTIMEOUT_S_NONBLOCK;
        select_timeout.tv_usec = kTIMEOUT_MS_NONBLOCK * 1000;

        n = select(sockfd + 1, NULL, &working_list, NULL, &select_timeout);

        if (n == -1) {
          std::cout << "ERROR " << strerror(errno) << ", on select at "
                    << __FILE__ << ":" << __LINE__ << std::endl;
          exit(0);
        }

        if (n == 1) break;
      }
    }

    n = ::write(sockfd, (char*)(&data_type) + size_written,
                size_to_write - size_written);

    if (n <= 0) {
      break;
    }

    size_written += n;
  }

  if (n < 0) {
    std::cout << "ERROR " << strerror(errno) << ", writing to socket at "
              << __FILE__ << ":" << __LINE__ << std::endl;
    ;
    return n;
  }

  // timespec
  size_to_write = sizeof(struct timespec);
  size_written = 0;

  while (size_written < size_to_write) {
    if (!blocking_client) {
      while (client_active.load()) {
        working_list = master_list;
        select_timeout.tv_sec = kTIMEOUT_S_NONBLOCK;
        select_timeout.tv_usec = kTIMEOUT_MS_NONBLOCK * 1000;

        n = select(sockfd + 1, NULL, &working_list, NULL, &select_timeout);

        if (n == -1) {
          std::cout << "ERROR " << strerror(errno) << ", on select at "
                    << __FILE__ << ":" << __LINE__ << std::endl;
          exit(0);
        }

        if (n == 1) break;
      }
    }

    n = ::write(sockfd, (char*)(ptms) + size_written,
                size_to_write - size_written);

    if (n <= 0) {
      break;
    }

    size_written += n;
  }

  if (free_ptms) {
    free(ptms);
    ptms = NULL;
  }

  if (n < 0) {
    std::cout << "ERROR " << strerror(errno) << ", writing to socket at "
              << __FILE__ << ":" << __LINE__ << std::endl;
    ;
    return n;
  }

  // size
  size_to_write = sizeof(size);
  size_written = 0;

  while (size_written < size_to_write) {
    if (!blocking_client) {
      while (client_active.load()) {
        working_list = master_list;
        select_timeout.tv_sec = kTIMEOUT_S_NONBLOCK;
        select_timeout.tv_usec = kTIMEOUT_MS_NONBLOCK * 1000;

        n = select(sockfd + 1, NULL, &working_list, NULL, &select_timeout);

        if (n == -1) {
          std::cout << "ERROR " << strerror(errno) << ", on select at "
                    << __FILE__ << ":" << __LINE__ << std::endl;
          exit(0);
        }

        if (n == 1) break;
      }
    }

    n = ::write(sockfd, (char*)(&size) + size_written,
                size_to_write - size_written);

    if (n <= 0) {
      break;
    }

    size_written += n;
  }

  if (n < 0) {
    std::cout << "ERROR " << strerror(errno) << ", writing to socket at "
              << __FILE__ << ":" << __LINE__ << std::endl;
    ;
    return n;
  }

  // data
  size_to_write = size;
  size_written = 0;

  while (size_written < size_to_write) {
    if (!blocking_client) {
      while (client_active.load()) {
        working_list = master_list;
        select_timeout.tv_sec = kTIMEOUT_S_NONBLOCK;
        select_timeout.tv_usec = kTIMEOUT_MS_NONBLOCK * 1000;

        n = select(sockfd + 1, NULL, &working_list, NULL, &select_timeout);

        if (n == -1) {
          std::cout << "ERROR " << strerror(errno) << ", on select at "
                    << __FILE__ << ":" << __LINE__ << std::endl;
          exit(0);
        }

        if (n == 1) break;
      }
    }

    n = ::write(sockfd, (char*)(buffer) + size_written,
                size_to_write - size_written);

    if (n <= 0) {
      break;
    }

    size_written += n;
  }

  if (n < 0) {
    std::cout << "ERROR " << strerror(errno) << ", writing to socket at "
              << __FILE__ << ":" << __LINE__ << std::endl;
    ;
    return n;
  }

  return size_written;
}

Client::~Client() {
  client_active.store(false);
  close(sockfd);
}

Server::Server(unsigned short port_, bool blocking) {
  int enable = 1;
  port = port_;
  blocking_server = blocking;

  if (!blocking_server) {
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(now);
    auto fraction = now - seconds;
    auto milliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(fraction);

    std::cout << "[WARNING] BF NONBLOCK SERVER @"
              << std::put_time(std::localtime(&now_c), "%c")
              << std::to_string(milliseconds.count()) << "\n";
  }

  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0) {
    std::cout << "ERROR " << strerror(errno) << ", opening socket at "
              << __FILE__ << ":" << __LINE__ << std::endl;
    exit(0);
  }

  if (!blocking_server) {
    if (fcntl(sockfd, F_SETFL, O_NONBLOCK) < 0) {
      std::cout << "ERROR " << strerror(errno)
                << ", setting socket to nonblock at " << __FILE__ << ":"
                << __LINE__ << std::endl;
      exit(0);
    }
  }

  if (setsockopt(sockfd, SOL_SOCKET, SO_REUSEADDR, &enable, sizeof(int)) < 0) {
    std::cout << "ERROR " << strerror(errno)
              << ", setting socket option (SO_REUSEADDR) at " << __FILE__ << ":"
              << __LINE__ << std::endl;
    exit(0);
  }

  bzero((char*)&serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = INADDR_ANY;
  serv_addr.sin_port = htons(port);
  if (bind(sockfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) < 0) {
    std::cout << "ERROR " << strerror(errno) << ", on binding at " << __FILE__
              << ":" << __LINE__ << std::endl;
    exit(0);
  }

  socklen_t len = sizeof(serv_addr);
  if (getsockname(sockfd, (struct sockaddr*)&serv_addr, &len) == -1) {
    std::cout << "ERROR " << strerror(errno) << ", on getsockname at "
              << __FILE__ << ":" << __LINE__ << std::endl;
    exit(0);
  } else
    port = ntohs(serv_addr.sin_port);

  // figure out the host_name
  char hostname_[128];
  gethostname(hostname_, 128);
  host_name = std::string(hostname_);

  if (!blocking_server) {
    FD_ZERO(&connection_working_list);
    FD_ZERO(&connection_master_list);

    FD_SET(sockfd, &connection_master_list);

    server_active.store(true);
    server_connected.store(false);
  }
}

bool Server::listen_to_client(int retry_count, std::atomic_bool* notify_bool) {
  listen(sockfd, 10);
  socklen_t clilen = sizeof(cli_addr);

  if (notify_bool) {
    notify_bool->store(true);
  }

  if (!blocking_server) {
    timeval select_timeout;
    select_timeout.tv_sec = kTIMEOUT_S_NONBLOCK;
    select_timeout.tv_usec = kTIMEOUT_MS_NONBLOCK * 1000;

    while (server_active.load()) {
      connection_working_list = connection_master_list;

      static int notification_counter = kNotificationWaitTime;
      notification_counter++;

      select_timeout.tv_sec = kTIMEOUT_S_NONBLOCK;
      select_timeout.tv_usec = kTIMEOUT_MS_NONBLOCK * 1000;

      int n = select(sockfd + 1, &connection_working_list, NULL, NULL,
                     &select_timeout);
      if (notification_counter > kNotificationWaitTime) {
        std::cout << "Accepting connections @ " << std::to_string(port)
                  << std::endl;
        notification_counter = 0;
      }

      if (n == -1) {
        std::cout << "ERROR " << strerror(errno) << ", on select at "
                  << __FILE__ << ":" << __LINE__ << std::endl;
        exit(0);
      }

      if (n == 1) {
        break;
      }

      retry_count--;
      // time out -- return false and let the caller call this again
      if (retry_count == 0) return false;
    }
  }

  newsockfd = accept(sockfd, (struct sockaddr*)&cli_addr, &clilen);
  if (newsockfd < 0) {
    // std::cout<<"ERROR on accept at "<<__FILE__<<":"<< __LINE__<<std::endl;
    return false;
  }

  if (!blocking_server) {
    if (fcntl(newsockfd, F_SETFL, O_NONBLOCK) < 0) {
      std::cout << "ERROR " << strerror(errno)
                << ", setting socket to nonblock at " << __FILE__ << ":"
                << __LINE__ << std::endl;
      exit(0);
    }

    FD_ZERO(&data_working_list);
    FD_ZERO(&data_master_list);

    FD_SET(newsockfd, &data_master_list);
    server_connected.store(true);
  }

  return true;
}

int Server::read(void** pbuffer, timespec* ptms) {
  int n;
  bool free_ptms = false;
  size_t size_of_msg = 0;

  if (ptms == NULL) {
    free_ptms = true;
    ptms = (timespec*)malloc(sizeof(timespec));
  }

  timeval select_timeout;
  select_timeout.tv_sec = kTIMEOUT_S_NONBLOCK;
  select_timeout.tv_usec = kTIMEOUT_MS_NONBLOCK * 1000;  // 500ms

  int size_to_read = 0;
  int size_read = 0;
  char data_type = 0;

  size_to_read = sizeof(data_type);
  size_read = 0;

  while (size_read < size_to_read) {
    if (!blocking_server) {
      while (server_connected.load()) {
        data_working_list = data_master_list;
        select_timeout.tv_sec = kTIMEOUT_S_NONBLOCK;
        select_timeout.tv_usec = kTIMEOUT_MS_NONBLOCK * 1000;

        n = select(newsockfd + 1, &data_working_list, NULL, NULL,
                   &select_timeout);

        if (n == -1) {
          std::cout << "ERROR " << strerror(errno) << ", on select at "
                    << __FILE__ << ":" << __LINE__ << std::endl;
          exit(0);
        }

        if (n == 1) break;
      }
    }

    n = ::read(newsockfd, (char*)(&data_type) + size_read,
               size_to_read - size_read);

    if (n < 0) {
      break;
    }

    size_read += n;
  }

  if (n < 0) {
    std::cout << "ERROR " << strerror(errno) << ", reading from socket at "
              << __FILE__ << ":" << __LINE__ << std::endl;
    return n;
  }

  if (data_type != kDATA_TYPE_DATA) {
    std::cout << "ERROR GOT AN INVALID DATA_TYPE IN SERVER::READ [" << data_type
              << "]" << std::endl
              << std::flush;
    n = -1;
    return n;
  }

  size_to_read = sizeof(struct timespec);
  size_read = 0;

  while (size_read < size_to_read) {
    if (!blocking_server) {
      while (server_connected.load()) {
        data_working_list = data_master_list;
        select_timeout.tv_sec = kTIMEOUT_S_NONBLOCK;
        select_timeout.tv_usec = kTIMEOUT_MS_NONBLOCK * 1000;

        n = select(newsockfd + 1, &data_working_list, NULL, NULL,
                   &select_timeout);

        if (n == -1) {
          std::cout << "ERROR " << strerror(errno) << ", on select at "
                    << __FILE__ << ":" << __LINE__ << std::endl;
          exit(0);
        }

        if (n == 1) break;
      }
    }

    n = ::read(newsockfd, (char*)(ptms) + size_read, size_to_read - size_read);

    if (n < 0) {
      break;
    }

    size_read += n;
  }

  if (free_ptms) {
    free(ptms);
    ptms = NULL;
  }

  if (n < 0) {
    std::cout << "ERROR " << strerror(errno) << ", reading from socket at "
              << __FILE__ << ":" << __LINE__ << std::endl;
    return n;
  }

  size_to_read = sizeof(size_of_msg);
  size_read = 0;

  while (size_read < size_to_read) {
    if (!blocking_server) {
      while (server_connected.load()) {
        data_working_list = data_master_list;
        select_timeout.tv_sec = kTIMEOUT_S_NONBLOCK;
        select_timeout.tv_usec = kTIMEOUT_MS_NONBLOCK * 1000;

        n = select(newsockfd + 1, &data_working_list, NULL, NULL,
                   &select_timeout);

        if (n == -1) {
          std::cout << "ERROR " << strerror(errno) << ", on select at "
                    << __FILE__ << ":" << __LINE__ << std::endl;
          exit(0);
        }

        if (n == 1) break;
      }
    }

    n = ::read(newsockfd, (char*)(&size_of_msg) + size_read,
               size_to_read - size_read);

    if (n < 0) {
      break;
    }

    size_read += n;
  }

  if (n < 0) {
    std::cout << "ERROR " << strerror(errno) << ", reading from socket at "
              << __FILE__ << ":" << __LINE__ << std::endl;
    return n;
  }

  if (size_of_msg == 0) return 0;

  if (*pbuffer == NULL) *pbuffer = malloc(size_of_msg);

  size_to_read = size_of_msg;
  size_read = 0;

  while (size_read < size_to_read) {
    if (!blocking_server) {
      while (server_connected.load()) {
        data_working_list = data_master_list;
        select_timeout.tv_sec = kTIMEOUT_S_NONBLOCK;
        select_timeout.tv_usec = kTIMEOUT_MS_NONBLOCK * 1000;

        n = select(newsockfd + 1, &data_working_list, NULL, NULL,
                   &select_timeout);

        if (n == -1) {
          std::cout << "ERROR " << strerror(errno) << ", on select at "
                    << __FILE__ << ":" << __LINE__ << std::endl;
          exit(0);
        }

        if (n == 1) break;
      }
    }

    n = ::read(newsockfd, (char*)(*pbuffer) + size_read,
               size_to_read - size_read);

    if (n < 0) {
      break;
    }

    size_read += n;
  }

  if (n <= 0) {
    if (n == 0) {
      std::cout << "ERROR " << strerror(errno) << ", reading from socket at "
                << __FILE__ << ":" << __LINE__ << " ZERO BYTE" << std::endl
                << std::flush;
      n = -1;
    } else {
      std::cout << "ERROR " << strerror(errno) << ", reading from socket at "
                << __FILE__ << ":" << __LINE__ << std::endl;
    }
    return n;
  }

  return size_of_msg;
}

int Server::write(const void* buffer, size_t size, timespec* ptms,
                  unsigned long width, unsigned long height,
                  unsigned int type) {
  int n;
  bool free_ptms = false;
  if (ptms == NULL) {
    free_ptms = true;
    ptms = (timespec*)malloc(sizeof(timespec));
    if (get_time(ptms)) {
      std::cout << "ERROR get_time";
      exit(0);
    }
  }

  char data_type = kDATA_TYPE_DATA;

  if (width != 0 || height != 0 || type != 0) {
    data_type = kDATA_TYPE_MATRIX;
  }

  timeval select_timeout;
  select_timeout.tv_sec = kTIMEOUT_S_NONBLOCK;
  select_timeout.tv_usec = kTIMEOUT_MS_NONBLOCK * 1000;  // 500ms

  int size_to_write = 0;
  int size_written = 0;

  // first send the data type (char)
  size_to_write = sizeof(data_type);
  size_written = 0;

  while (size_written < size_to_write) {
    if (!blocking_server) {
      while (server_connected.load()) {
        data_working_list = data_master_list;
        select_timeout.tv_sec = kTIMEOUT_S_NONBLOCK;
        select_timeout.tv_usec = kTIMEOUT_MS_NONBLOCK * 1000;

        n = select(newsockfd + 1, NULL, &data_working_list, NULL,
                   &select_timeout);

        if (n == -1) {
          std::cout << "ERROR " << strerror(errno) << ", on select at "
                    << __FILE__ << ":" << __LINE__ << std::endl;
          exit(0);
        }

        if (n == 1) break;
      }
    }

    n = ::write(newsockfd, &data_type, size_to_write - size_written);

    if (n <= 0) {
      break;
    }

    size_written += n;
  }

  if (n < 0) {
    std::cout << "ERROR " << strerror(errno) << ", writing to socket at "
              << __FILE__ << ":" << __LINE__ << std::endl;
    ;
    return n;
  }

  if (data_type == kDATA_TYPE_MATRIX) {
    // send 3 extra fields -- width
    size_to_write = sizeof(width);
    size_written = 0;

    while (size_written < size_to_write) {
      if (!blocking_server) {
        while (server_connected.load()) {
          data_working_list = data_master_list;
          select_timeout.tv_sec = kTIMEOUT_S_NONBLOCK;
          select_timeout.tv_usec = kTIMEOUT_MS_NONBLOCK * 1000;

          n = select(newsockfd + 1, NULL, &data_working_list, NULL,
                     &select_timeout);

          if (n == -1) {
            std::cout << "ERROR " << strerror(errno) << ", on select at "
                      << __FILE__ << ":" << __LINE__ << std::endl;
            exit(0);
          }

          if (n == 1) break;
        }
      }

      n = ::write(newsockfd, (char*)(&width) + size_written,
                  size_to_write - size_written);

      if (n <= 0) {
        break;
      }

      size_written += n;
    }

    if (n < 0) {
      std::cout << "ERROR " << strerror(errno) << ", writing to socket at "
                << __FILE__ << ":" << __LINE__ << std::endl;
      ;
      return n;
    }

    // send 3 extra fields -- height
    size_to_write = sizeof(height);
    size_written = 0;

    while (size_written < size_to_write) {
      if (!blocking_server) {
        while (server_connected.load()) {
          data_working_list = data_master_list;
          select_timeout.tv_sec = kTIMEOUT_S_NONBLOCK;
          select_timeout.tv_usec = kTIMEOUT_MS_NONBLOCK * 1000;

          n = select(newsockfd + 1, NULL, &data_working_list, NULL,
                     &select_timeout);

          if (n == -1) {
            std::cout << "ERROR " << strerror(errno) << ", on select at "
                      << __FILE__ << ":" << __LINE__ << std::endl;
            exit(0);
          }

          if (n == 1) break;
        }
      }

      n = ::write(newsockfd, (char*)(&height) + size_written,
                  size_to_write - size_written);

      if (n <= 0) {
        break;
      }

      size_written += n;
    }

    if (n < 0) {
      std::cout << "ERROR " << strerror(errno) << ", writing to socket at "
                << __FILE__ << ":" << __LINE__ << std::endl;
      ;
      return n;
    }

    // send 3 extra fields -- type
    size_to_write = sizeof(type);
    size_written = 0;

    while (size_written < size_to_write) {
      if (!blocking_server) {
        while (server_connected.load()) {
          data_working_list = data_master_list;
          select_timeout.tv_sec = kTIMEOUT_S_NONBLOCK;
          select_timeout.tv_usec = kTIMEOUT_MS_NONBLOCK * 1000;

          n = select(newsockfd + 1, NULL, &data_working_list, NULL,
                     &select_timeout);

          if (n == -1) {
            std::cout << "ERROR " << strerror(errno) << ", on select at "
                      << __FILE__ << ":" << __LINE__ << std::endl;
            exit(0);
          }

          if (n == 1) break;
        }
      }

      n = ::write(newsockfd, (char*)(&type) + size_written,
                  size_to_write - size_written);

      if (n <= 0) {
        break;
      }

      size_written += n;
    }

    if (n < 0) {
      std::cout << "ERROR " << strerror(errno) << ", writing to socket at "
                << __FILE__ << ":" << __LINE__ << std::endl;
      ;
      return n;
    }
  }

  // send the time of the data capture (struct timespec)
  size_to_write = sizeof(struct timespec);
  size_written = 0;

  while (size_written < size_to_write) {
    if (!blocking_server) {
      while (server_connected.load()) {
        data_working_list = data_master_list;
        select_timeout.tv_sec = kTIMEOUT_S_NONBLOCK;
        select_timeout.tv_usec = kTIMEOUT_MS_NONBLOCK * 1000;

        n = select(newsockfd + 1, NULL, &data_working_list, NULL,
                   &select_timeout);

        if (n == -1) {
          std::cout << "ERROR " << strerror(errno) << ", on select at "
                    << __FILE__ << ":" << __LINE__ << std::endl;
          exit(0);
        }

        if (n == 1) break;
      }
    }

    n = ::write(newsockfd, (char*)(ptms) + size_written,
                size_to_write - size_written);

    if (n <= 0) {
      break;
    }

    size_written += n;
  }

  if (free_ptms) {
    free(ptms);
    ptms = NULL;
  }

  if (n < 0) {
    std::cout << "ERROR " << strerror(errno) << ", writing to socket at "
              << __FILE__ << ":" << __LINE__ << std::endl;
    ;
    return n;
  }

  // send the size of the data (size_t)
  size_to_write = sizeof(size);
  size_written = 0;

  while (size_written < size_to_write) {
    if (!blocking_server) {
      while (server_connected.load()) {
        data_working_list = data_master_list;
        select_timeout.tv_sec = kTIMEOUT_S_NONBLOCK;
        select_timeout.tv_usec = kTIMEOUT_MS_NONBLOCK * 1000;

        n = select(newsockfd + 1, NULL, &data_working_list, NULL,
                   &select_timeout);

        if (n == -1) {
          std::cout << "ERROR " << strerror(errno) << ", on select at "
                    << __FILE__ << ":" << __LINE__ << std::endl;
          exit(0);
        }

        if (n == 1) break;
      }
    }

    n = ::write(newsockfd, (char*)(&size) + size_written,
                size_to_write - size_written);

    if (n <= 0) {
      break;
    }

    size_written += n;
  }

  if (n < 0) {
    std::cout << "ERROR " << strerror(errno) << ", writing to socket at "
              << __FILE__ << ":" << __LINE__ << std::endl;
    ;
    return n;
  }

  // send the data (void*)
  size_to_write = size;
  size_written = 0;

  char* buffer_ptr = (char*)buffer;

  while (size_written < size_to_write) {
    if (!blocking_server) {
      while (server_connected.load()) {
        data_working_list = data_master_list;
        select_timeout.tv_sec = kTIMEOUT_S_NONBLOCK;
        select_timeout.tv_usec = kTIMEOUT_MS_NONBLOCK * 1000;

        n = select(newsockfd + 1, NULL, &data_working_list, NULL,
                   &select_timeout);

        if (n == -1) {
          std::cout << "ERROR " << strerror(errno) << ", on select at "
                    << __FILE__ << ":" << __LINE__ << std::endl;
          exit(0);
        }

        if (n == 1) break;
      }
    }

    n = ::write(newsockfd, buffer_ptr + size_written,
                size_to_write - size_written);

    if (n < 0) {
      break;
    }

    size_written += n;
  }

  if (n < 0) {
    std::cout << "ERROR " << strerror(errno) << ", writing to socket at "
              << __FILE__ << ":" << __LINE__ << std::endl;
    ;
    return n;
  }

  return size_written;
}

void Server::close_client() {
  server_connected.store(false);
  close(newsockfd);
}

Server::~Server() {
  server_active.store(false);
  close(sockfd);
}

bool rpc(std::string host_name, unsigned short host_port,
         const void* send_msg_ptr, size_t send_msg_len, void** receive_msg_buf,
         int* receive_msg_buf_size) {
  bool connectionSucceed;
  Client client(host_name, host_port, true, &connectionSucceed);
  if (connectionSucceed) {
    if (client.write(send_msg_ptr, send_msg_len) >= 0) {
      *receive_msg_buf_size = client.read(receive_msg_buf);
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

bool nonblock_rpc(std::string host_name, unsigned short host_port,
                  const void* send_msg_ptr, size_t send_msg_len,
                  void** receive_msg_buf, int* receive_msg_buf_size) {
  bool connectionSucceed;
  Client client(host_name, host_port, false, &connectionSucceed);
  if (connectionSucceed) {
    if (client.write(send_msg_ptr, send_msg_len) >= 0) {
      *receive_msg_buf_size = client.read(receive_msg_buf);
      return true;
    } else {
      return false;
    }
  } else {
    return false;
  }
}

void run_master(unsigned short port) {
  signal(SIGPIPE, SIG_IGN);

  std::vector<std::string> publisher_names;
  std::vector<std::string> publisher_conns;
  Server server(port, true);  // master is always non blocking

  std::ifstream existing_publishers(".publishers");
  std::string publisher_line;
  size_t pos_bar;
  size_t pos_at;

  if (existing_publishers.is_open()) {
    std::cout.setstate(std::ios_base::failbit);  // squelch cout

    while (std::getline(existing_publishers, publisher_line)) {
      pos_bar = publisher_line.find_first_of('|');
      pos_at = publisher_line.find_first_of('@');

      // try to establish connection to the existing publisher
      std::string publisher_name = publisher_line.substr(0, pos_bar);
      std::string publisher_hostname =
          publisher_line.substr(pos_bar + 1, pos_at - pos_bar - 1);
      unsigned short publisher_listen_port =
          std::stoi(publisher_line.substr(pos_at + 1));

      std::string verification_msg = "VER:" + publisher_name;

      char* msg2receiveFromPublisher = NULL;
      int msg2receiveSize = 0;
      if (rpc(publisher_hostname, publisher_listen_port,
              verification_msg.c_str(), verification_msg.length() + 1,
              (void**)(&msg2receiveFromPublisher), &msg2receiveSize)) {
        if (msg2receiveSize > 0) {
          std::string str2receiveFromPublisher(msg2receiveFromPublisher);
          free(msg2receiveFromPublisher);

          if (str2receiveFromPublisher.compare(verification_msg) == 0) {
            publisher_names.push_back(publisher_line.substr(0, pos_bar));
            publisher_conns.push_back(publisher_line.substr(pos_bar + 1));

            std::cout.clear();  // restore cout
            std::cout << "Adding Existing Publisher: " << publisher_name << "|"
                      << publisher_hostname << "@" << publisher_listen_port
                      << std::endl;
            std::cout.setstate(std::ios_base::failbit);  // squelch cout
          }
        }
      }
    }

    existing_publishers.close();

    std::cout.clear();  // restore cout
  }

  // write all the publishers to the file again
  std::ofstream publishers_file(".publishers", std::ios_base::trunc);
  for (int i = 0; i < publisher_names.size(); i++) {
    publishers_file << publisher_names[i] << "|" << publisher_conns[i]
                    << std::endl;
  }
  publishers_file.close();

  for (;;) {
    if (server.listen_to_client()) {
      char* buffer = NULL;
      if (server.read((void**)(&buffer)) >= 0) {
        // a publisher publishing service, return suggested port number
        std::string in_msg(buffer);
        free(buffer);
        std::string result_str;

        std::cout << in_msg << std::endl;
        if (in_msg.compare(0, 4, "PUB:") == 0) {  // PUB:name|hostname:port
          pos_bar = in_msg.find_first_of('|');
          std::string pname = in_msg.substr(4, pos_bar - 4);
          int existing_idx = -1;

          // if this publisher exists remove it from the file
          for (int i = 0; i < publisher_names.size(); i++) {
            if (pname.compare(publisher_names[i]) == 0) {
              existing_idx = i;
              break;
            }
          }

          if (existing_idx >= 0) {
            std::cout << " Removing Publisher ("
                      << publisher_names[existing_idx] << ") @ ["
                      << publisher_conns[existing_idx] << "]" << std::endl;

            publisher_names.erase(publisher_names.begin() + existing_idx);
            publisher_conns.erase(publisher_conns.begin() + existing_idx);
            int i = 0;

            std::ifstream in_publishers(".publishers", std::ios_base::in);
            if (in_publishers.is_open()) {
              std::string in_line;
              std::ofstream out_publishers(".publisherstmp",
                                           std::ios_base::trunc);
              while (std::getline(in_publishers, in_line)) {
                if (i++ != existing_idx) {
                  out_publishers << in_line;
                }
              }

              out_publishers.close();
              in_publishers.close();
              std::remove(".publishers");
              std::rename(".publisherstmp", ".publishers");
            }
          }

          publisher_names.push_back(in_msg.substr(4, pos_bar - 4));
          publisher_conns.push_back(in_msg.substr(pos_bar + 1));

          result_str = "SUCCESS";

          std::cout << " Adding Publisher ("
                    << publisher_names[publisher_names.size() - 1] << ") @ ["
                    << publisher_conns[publisher_conns.size() - 1] << "]"
                    << std::endl;

          std::ofstream publishers(".publishers", std::ios_base::app);
          publishers << in_msg.substr(4) << std::endl;
          publishers.close();
        } else if (in_msg.compare(0, 4, "SUB:") == 0) {  // SUB:publisher_name
          std::string pname = in_msg.substr(4);

          std::cout << " Finding publisher (" << pname << ") ";

          int i = publisher_names.size() - 1;
          for (; i >= 0; --i) {
            // std::cout<<"i="<<i<<" publisher_name[i]="<<publisher_name[i]<<"
            // pname="<<pname<<std::endl;
            if (pname.compare(publisher_names[i]) == 0) break;
          }
          if (i < 0)
            result_str = "ERROR: Cannot find the publisher (" + pname + ")";
          else
            result_str = "FOUND:" + publisher_conns[i];

          static int notification_counter = kNotificationWaitTime;
          notification_counter++;
          if (notification_counter > kNotificationWaitTime * 100000) {
            std::cout << " " << result_str << std::endl;
            notification_counter = 0;
          }

        } else if (in_msg.compare(0, 4, "CAMS") == 0) {  // CAMS

          for (int i = 0; i < publisher_names.size(); i++) {
            if (publisher_names[i].find("cam") != std::string::npos ||
                publisher_names[i].find("Cam") != std::string::npos ||
                publisher_names[i].find("SN") != std::string::npos) {
              result_str += publisher_names[i] + "\n";
            }
          }

        } else {
          std::cout << " ERROR: Invalid message" << std::endl;
        }
        server.write(result_str.c_str(), result_str.length() + 1);
      }
    }

    server.close_client();
  }
}

void Publisher::lock() { mu.lock(); }

void Publisher::unlock() { mu.unlock(); }

void Publisher::listen() {
  for (;;) {
    if (!actively_listening.load()) break;

    if (subscription_server->listen_to_client()) {
      char* buffer = NULL;
      if (subscription_server->read((void**)(&buffer)) <= 0) continue;

      // a publisher publishing service, return suggested port number
      std::string in_msg(buffer);
      free(buffer);

      std::cout << in_msg << std::endl;

      std::string result_str;

      if (in_msg.compare(0, 4, "SUB:") == 0) {  // SUB:publisher_name
        if (!shm_publisher) {
          Server* data_server = new Server();  // data connection = nonblocking
          result_str = "PUB:" + publisher_name + "|" + data_server->host_name +
                       "@" + std::to_string(data_server->port);

          std::cout << result_str << std::endl;

          ready_to_notify_subscriber.store(false);

          listen_return = std::async(
              std::launch::async, &Server::listen_to_client, data_server,
              kPUBLISHER_DATA_SERVER_CONNECTION_RETRY_COUNT,
              &ready_to_notify_subscriber);

          while (!ready_to_notify_subscriber.load()) {
            usleep(kPUBLISHER_TIME_TO_SLEEP_BETWEEN_LISTEN_READY_MS *
                   1000);  // sleep
          }

          subscription_server->write(result_str.c_str(),
                                     result_str.length() + 1);
          subscription_server->close_client();

          if (listen_return.get()) {
            // block until the client connect me to finish the hand shaking
            // initialization
            lock();
            data_servers.push_back(data_server);
            std::cout << "successfully connected to subscriber at port "
                      << std::to_string(data_server->port) << std::endl;
            unlock();
          } else {
            delete data_server;
          }
        } else {
          SHMServer* shm_data_server = new SHMServer(publisher_name, shm_size);
          result_str = "PUB:" + publisher_name + "|" +
                       shm_data_server->host_name + "@" +
                       std::to_string(shm_data_server->port) + "&" +
                       std::to_string(shm_size);

          std::cout << result_str << std::endl;

          ready_to_notify_subscriber.store(false);
          listen_return = std::async(
              std::launch::async, &SHMServer::listen_to_client, shm_data_server,
              kPUBLISHER_DATA_SERVER_CONNECTION_RETRY_COUNT,
              &ready_to_notify_subscriber);

          while (!ready_to_notify_subscriber.load()) {
            usleep(kPUBLISHER_TIME_TO_SLEEP_BETWEEN_LISTEN_READY_MS * 1000);
          }

          subscription_server->write(result_str.c_str(),
                                     result_str.length() + 1);
          subscription_server->close_client();

          if (listen_return.get()) {
            lock();
            shm_data_servers.push_back(shm_data_server);
            std::cout << "successfully connected to subscriber at port  "
                      << std::to_string(shm_data_server->port)
                      << " over shared memory" << std::endl;
            unlock();
          } else {
            delete shm_data_server;
          }
        }
      } else if (in_msg.compare(0, 4, "VER:") == 0) {  // VER:publisher_name
        result_str = "VER:" + publisher_name;
        std::cout << result_str << std::endl;
        subscription_server->write(result_str.c_str(), result_str.length() + 1);
        subscription_server->close_client();
      } else {
        std::cout << "ERROR: Invalid message" << std::endl;
        result_str = "ERROR: Invalid message";
        subscription_server->write(result_str.c_str(), result_str.length() + 1);
        subscription_server->close_client();
      }
    }
  }
}

Publisher::Publisher(std::string publisher_name_, std::string master_name,
                     unsigned long _shm_size, unsigned short master_port,
                     bool listen_blocking) {
  signal(SIGPIPE, SIG_IGN);

  publisher_name = publisher_name_;
  // listen_to_subscriber
  subscription_server = new Server(
      0,
      listen_blocking);  // listen connection = listen_blocking (default = true)

  shm_publisher = kUSE_SHM && (_shm_size > 0);
  shm_size = _shm_size;

  std::string msg2send = "PUB:" + publisher_name + "|" +
                         subscription_server->host_name + "@" +
                         std::to_string(subscription_server->port);
  std::cout << msg2send << std::endl;
  char* msg2receive = NULL;
  int msg2receiveSize = 0;
  rpc(master_name, master_port, msg2send.c_str(), msg2send.length() + 1,
      (void**)(&msg2receive), &msg2receiveSize);

  if (msg2receiveSize <= 0) {
    std::cout << "Fatal error reading message from master " << __FILE__ << ":"
              << __LINE__ << std::endl;
    exit(0);
  }

  if (strcmp(msg2receive, "SUCCESS") != 0) {
    std::cout << "Fatal error" << std::endl;
    exit(0);
  }
  std::cout << "Publish servce [" << publisher_name
            << "] successfully to master [" << master_name << "]." << std::endl;
  free(msg2receive);

  actively_listening.store(true);

  listen_thread = std::thread(&Publisher::listen, this);
}

void Publisher::publish(void* buffer, unsigned long width, unsigned long height,
                        unsigned int elemSize, unsigned int matrix_type,
                        timespec* ptms) {
  lock();
  static int notification_counter = kNotificationWaitTime;
  notification_counter++;
  if (notification_counter > kNotificationWaitTime) {
    notification_counter = 0;
    std::cout << "[" << publisher_name << "] Sending to " << data_servers.size()
              << " subscribers" << std::endl;
  }
  unsigned long buffer_size = width * height * elemSize;
  if (!shm_publisher) {
    for (int i = data_servers.size() - 1; i >= 0; --i) {
      int n = data_servers[i]->write(buffer, buffer_size, ptms, width, height,
                                     matrix_type);
      // std::cout << "wrote packet of size: " << n << std::endl;
      if (n < 0) {
        data_servers[i]->close_client();

        Server* data_server = data_servers[i];
        delete data_server;

        data_servers.erase(data_servers.begin() + i);
        // std::cout << "failed sending \n";
      }
    }
  } else {
    for (int i = shm_data_servers.size() - 1; i >= 0; --i) {
      int n = shm_data_servers[i]->write(buffer, buffer_size, ptms, width,
                                         height, matrix_type);
      if (n < 0) {
        shm_data_servers[i]->close_client();

        SHMServer* data_server = shm_data_servers[i];
        delete data_server;

        shm_data_servers.erase(shm_data_servers.begin() + i);
      }
    }
  }
  unlock();
}

void Publisher::publish(void* buffer, size_t size, timespec* ptms) {
  lock();
  static int notification_counter = kNotificationWaitTime;
  notification_counter++;
  if (notification_counter > kNotificationWaitTime) {
    notification_counter = 0;
    std::cout << "[" << publisher_name << "] Sending to " << data_servers.size()
              << " subscribers" << std::endl;
  }

  if (!shm_publisher) {
    for (int i = data_servers.size() - 1; i >= 0; --i) {
      int n = data_servers[i]->write(buffer, size, ptms);
      // std::cout << "wrote packet of size: " << n << std::endl;
      if (n < 0) {
        data_servers[i]->close_client();

        Server* data_server = data_servers[i];
        delete data_server;

        data_servers.erase(data_servers.begin() + i);
        // std::cout << "failed sending \n";
      }
    }
  } else {
    for (int i = shm_data_servers.size() - 1; i >= 0; --i) {
      int n = shm_data_servers[i]->write(buffer, size, ptms);
      if (n < 0) {
        shm_data_servers[i]->close_client();

        SHMServer* data_server = shm_data_servers[i];
        delete data_server;

        shm_data_servers.erase(shm_data_servers.begin() + i);
      }
    }
  }
  unlock();
}

void Publisher::publish(char* buffer, size_t size, timespec* ptms) {
  this->publish((void*)buffer, size, ptms);
}

void Publisher::publish(double* buffer, size_t size, timespec* ptms) {
  this->publish((void*)buffer, size, ptms);
}

void Publisher::publish(int* buffer, size_t size, timespec* ptms) {
  this->publish((void*)buffer, size, ptms);
}

void Publisher::publish(float* buffer, size_t size, timespec* ptms) {
  this->publish((void*)buffer, size, ptms);
}

void Publisher::publish(bool* buffer, size_t size, timespec* ptms) {
  this->publish((void*)buffer, size, ptms);
}

Publisher::~Publisher() {
  actively_listening.store(false);

  listen_thread.join();

  delete subscription_server;
  lock();
  for (size_t i = 0; i < data_servers.size(); ++i) {
    data_servers[i]->close_client();
    delete data_servers[i];
  }
  for (size_t i = 0; i < shm_data_servers.size(); ++i) {
    shm_data_servers[i]->close_client();
    delete shm_data_servers[i];
  }
  unlock();
}

void Subscriber::lock() { mu.lock(); }

void Subscriber::unlock() { mu.unlock(); }

Datum Subscriber::get_the_lastest_and_clear_all() {  // always the latest
  initialize();
  Datum ret;
  ret.data = NULL;
  ret.size = 0;

  if (initialized) {
    if (!buffer.empty()) {
      lock();

      ret = buffer[buffer.size() - 1];
      for (size_t i = 0; i < buffer.size() - 1; ++i) {
        if (buffer[i].data != NULL) {
          free(buffer[i].data);
        }
      }
      buffer.clear();

      // std::cout<<"buffer size = "<< buffer.size()<<std::endl;
      unlock();
    }
  }

  return ret;
}

Datum Subscriber::pop_first() {  // FIFO
  initialize();
  Datum ret;
  if (!initialized || buffer.empty()) {
    ret.data = NULL;
    ret.size = 0;
  } else {
    lock();
    ret = buffer[0];
    buffer.erase(buffer.begin());

    // for(int i=0;i<10;i++){
    //  std::this_thread::sleep_for(std::chrono::milliseconds(30));
    //  std::cout<<"blocking you "<< i <<" ... = "<< buffer.size()<<std::endl;
    //}
    unlock();
  }

  return ret;
}

Datum Subscriber::pop_last() {  // Stack FILO
  initialize();
  Datum ret;
  if (!initialized || buffer.empty()) {
    ret.data = NULL;
    ret.size = 0;
  } else {
    lock();
    ret = buffer[buffer.size() - 1];
    // buffer.erase(buffer.end()-1);
    buffer.erase(buffer.begin() + buffer.size() - 1);

    // std::cout<<"buffer size = "<< buffer.size()<<std::endl;
    unlock();
  }

  return ret;
}

Datum Subscriber::sync_read() {
  for (;;) {
    initialize();
    if (!initialized || buffer.empty()) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    } else {
      return get_the_lastest_and_clear_all();
    }
  }
}

void Subscriber::listen() {
  while (actively_listening.load()) {
    if (initialized) {
      if (shmClient == NULL) {
        // std::cout<<"bf: receiving ";
        Datum d;
        d.data = NULL;
        int n;

        n = client->read(&d.data, &d.time, &d.data_type, &d.width, &d.height,
                         &d.matrix_type);
        // std::cout << n << " bytes " << std::endl << std::flush;
        if (n > 0) {
          d.size = n;

          lock();
          buffer.push_back(d);
          // std::cout<< " buffer size = " <<buffer.size()<<std::endl;
          unlock();
        } else {
          initialized = false;
          // need to close the old socket here?
        }
      } else {
        if (!shmClient->isActive()) {
          initialized = false;
        } else {
          Datum d;
          d.data = NULL;
          d.size = 0;
          int n;

          n = shmClient->read(&d.data, &d.time, &d.data_type, &d.width,
                              &d.height, &d.matrix_type);
          if (n > 0) {
            d.size = n;

            lock();
            buffer.push_back(d);
            unlock();
          }
        }
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
      }
    } else {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }
  }
}

void Subscriber::initialize() {
  signal(SIGPIPE, SIG_IGN);

  if (!initialized) {
    std::string msg2send = "SUB:" + publisher_name;

    if (client) {
      // std::cout << "deleted client \n";
      delete client;
      client = NULL;

      lock();
      for (size_t i = 0; i < buffer.size(); ++i)
        if (buffer[i].data != NULL) free(buffer[i].data);

      buffer.clear();

      // std::cout<<"buffer size = "<< buffer.size()<<std::endl;
      unlock();
    }

    if (shmClient) {
      delete shmClient;
      shmClient = NULL;

      // no data to delete here
    }

    char* msg2receive = NULL;
    int msg2receiveSize = 0;
    if (rpc(master_name, master_port, msg2send.c_str(), msg2send.length() + 1,
            (void**)(&msg2receive), &msg2receiveSize)) {
      if (msg2receiveSize > 0) {
        std::string str2receive(msg2receive);
        free(msg2receive);

        if (str2receive.compare(0, 6, "FOUND:") == 0) {
          size_t pos_bar = str2receive.find_first_of('@');
          std::string publisher_hostname = str2receive.substr(6, pos_bar - 6);

          unsigned short publisher_listen_port =
              std::stoi(str2receive.substr(pos_bar + 1));

          static int notification_counter = kNotificationWaitTime;
          notification_counter++;
          if (notification_counter > kNotificationWaitTime * 1000000) {
            notification_counter = 0;
            std::cout << str2receive << " publisher_hostname=["
                      << publisher_hostname << "] publisher_listen_port=["
                      << publisher_listen_port << "] "
                      << "publisher_name = [" << publisher_name << "]"
                      << std::endl;
          }

          // connect to the publisher to get a new port for streaming data
          char* msg2receiveFromPublisher = NULL;
          int msg2receiveFromPublisherSize = 0;

          if (rpc(publisher_hostname, publisher_listen_port, msg2send.c_str(),
                  msg2send.length() + 1, (void**)(&msg2receiveFromPublisher),
                  &msg2receiveFromPublisherSize)) {
            if (msg2receiveFromPublisherSize > 0) {
              std::string str2receiveFromPublisher(msg2receiveFromPublisher);
              free(msg2receiveFromPublisher);

              size_t pos_at = str2receiveFromPublisher.find_first_of('@');
              size_t pos_and =
                  str2receiveFromPublisher.find_first_of('&', pos_at);

              if (pos_and == std::string::npos) {
                unsigned short publisher_data_port =
                    std::stoi(str2receiveFromPublisher.substr(pos_at + 1));

                std::cout << "P2P connection to Publisher [" << publisher_name
                          << "] at port [" << publisher_data_port << "]"
                          << std::endl;

                client = new Client(
                    publisher_hostname,
                    publisher_data_port);  // data connection = nonblocking
              } else {
                unsigned short publisher_data_port =
                    std::stoi(str2receiveFromPublisher.substr(
                        pos_at + 1, pos_and - pos_at));
                unsigned long shm_size =
                    std::stoi(str2receiveFromPublisher.substr(pos_and + 1));
                std::string shm_name = kSHM_PREFIX + publisher_name +
                                       std::to_string(publisher_data_port) +
                                       ".shm";

                shmClient =
                    new SHMClient(publisher_hostname, publisher_data_port,
                                  shm_name, shm_size);
              }
              initialized = true;
            }
          }
        }
      }
    }
  }
}

Subscriber::Subscriber(std::string publisher_name_, std::string master_name_,
                       unsigned short master_port_) {
  publisher_name = publisher_name_;
  master_name = master_name_;
  master_port = master_port_;
  initialized = false;

  client = NULL;
  shmClient = NULL;

  initialize();

  actively_listening.store(true);

  // create a thread to always listening to data
  listen_thread = std::thread(&Subscriber::listen, this);
}

Subscriber::~Subscriber() {
  actively_listening.store(false);

  listen_thread.join();

  if (client) delete client;
}

SHMClient::SHMClient(std::string host_name, unsigned short host_port,
                     std::string SHMname, unsigned long SHMsize, bool blocking,
                     bool* pResult) {
  struct sockaddr_in serv_addr;
  struct hostent* server;
  sockfd = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd < 0) {
    std::cout << "ERROR " << strerror(errno) << ", opening socket at "
              << __FILE__ << ":" << __LINE__ << std::endl;
    exit(0);
  }

  if (blocking && (pResult == NULL)) {
    std::cout << "ERROR cannot have a blocking client and no pResult "
              << std::endl;
    exit(0);
  }

  blocking_client = blocking;
  if (!blocking_client) {
    if (fcntl(sockfd, F_SETFL, O_NONBLOCK) < 0) {
      std::cout << "ERROR " << strerror(errno)
                << ", setting socket to nonblock at " << __FILE__ << ":"
                << __LINE__ << std::endl;
      exit(0);
    }
  }

  bzero((char*)&serv_addr, sizeof(serv_addr));
  server = gethostbyname(host_name.c_str());
  serv_addr.sin_family = AF_INET;
  if (server == NULL) {
    std::cout << "Cannot find such host, trying to connect via IP";
    serv_addr.sin_addr.s_addr = inet_addr(host_name.c_str());
  } else {
    bcopy((char*)server->h_addr, (char*)&serv_addr.sin_addr.s_addr,
          server->h_length);
  }
  serv_addr.sin_port = htons(host_port);

  bool retVal = true;

  int result = 0;

  result = connect(sockfd, (struct sockaddr*)&serv_addr, sizeof(serv_addr));
  if (result < 0) {
    if (blocking_client) {
      retVal = false;
      static int notification_counter = kNotificationWaitTime;
      notification_counter++;
      if (notification_counter > kNotificationWaitTime) {
        notification_counter = 0;
        std::cout << "ERROR " << strerror(errno) << ", connecting at "
                  << __FILE__ << ":" << __LINE__ << std::endl;
      }
    } else {
      retVal = false;

      if (errno == EINPROGRESS) {
        fd_set temp_working_list;
        fd_set temp_master_list;
        FD_ZERO(&temp_working_list);
        FD_ZERO(&temp_master_list);
        FD_SET(sockfd, &temp_master_list);

        temp_working_list = temp_master_list;
        result = select(sockfd + 1, NULL, &temp_working_list, NULL, NULL);

        if (result < 0) {
          std::cout << "ERROR " << strerror(errno) << ", on select at "
                    << __FILE__ << ":" << __LINE__ << std::endl;
          goto after_select;
        }

        socklen_t result_len = sizeof(result);
        if (getsockopt(sockfd, SOL_SOCKET, SO_ERROR, &result, &result_len) <
            0) {
          std::cout << "ERROR " << strerror(errno)
                    << ", connecting to NONBLOCK socket at " << __FILE__ << ":"
                    << __LINE__ << std::endl;
          goto after_select;
        }

        if (result == 0) {
          // connected succesfully -- retVal = true;
          // std::cout << "SOERROR = SUCCESS \n";
          retVal = true;
        } else {
          std::cout << "ERROR [" << strerror(result)
                    << "] connecting to NONBLOCK socket at " << __FILE__ << ":"
                    << __LINE__ << std::endl;
          auto now = std::chrono::system_clock::now();
          auto now_c = std::chrono::system_clock::to_time_t(now);
          auto seconds =
              std::chrono::time_point_cast<std::chrono::seconds>(now);
          auto fraction = now - seconds;
          auto milliseconds =
              std::chrono::duration_cast<std::chrono::milliseconds>(fraction);

          std::cout << "BF FATAL FAILURE NONBLOCK CLIENT @ "
                    << std::put_time(std::localtime(&now_c), "%c")
                    << std::to_string(milliseconds.count()) << "\n";
        }
      }  // errno == EIN_PROGRESS

    }  // if blocking_client
  }    // if connect result < 0

after_select:
  if (pResult != NULL) *pResult = retVal;

  if (!retVal) {
    if (pResult) {
      return;
    }

    std::cout << "FAILURE CONNECTING TO " << std::to_string(host_port)
              << std::endl;
    std::cout << "PLEASE DO NOT PRESS CTRL+C AND WAIT 5 SECONDS FOR THE "
                 "BINARY TO EXIT GRACEFULLY!!"
              << std::endl;
    std::cout << "YOU CAN RELAUNCH AFTER THE BINARY EXITS. THANK YOU"
              << std::endl;
    usleep(kSLEEP_AFTER_NONBLOCK_FAIL_S * 1000000);
    exit(0);
  }

  if ((shm_key = ftok(SHMname.c_str(), 'R')) == -1) {
    std::cout << "FAILURE CONNECTING TO SHARED MEMORY : " << SHMname
              << std::endl;
    close(sockfd);
    exit(0);
  }

  shm_size = SHMsize;

  if ((shm_id = shmget(shm_key, SHMsize, 0644 | IPC_CREAT)) == -1) {
    std::cout << "FAILURE GETTING SHARED MEMORY : " << SHMname << std::endl;
    close(sockfd);
    exit(0);
  }

  shm = (char*)shmat(shm_id, (void*)0, 0);
  if (shm == (char*)-1) {
    std::cout << "FAILURE MAPPING SHARED MEMORY : " << SHMname << std::endl;
    close(sockfd);
    exit(0);
  }

  cur_loc = 0;
  got_first_msg = false;
  last_packet_size = 0;

  // todo CONNECT semaphore

  // successful here
  if (!blocking_client) {
    FD_ZERO(&master_list);
    FD_ZERO(&working_list);
    FD_SET(sockfd, &master_list);
  }

  client_active.store(true);

  // start listening thread
  incoming_message_thread =
      std::thread(&SHMClient::check_for_incoming_message, this);
}

bool SHMClient::isActive() { return client_active.load(); }

void SHMClient::check_for_incoming_message() {
  timeval select_timeout;
  select_timeout.tv_sec = kTIMEOUT_S_NONBLOCK;
  select_timeout.tv_usec = kTIMEOUT_MS_NONBLOCK * 1000;  // 500ms

  while (client_active.load()) {
    int incoming_packet_size = 0;
    int size_to_read = sizeof(incoming_packet_size);
    int size_read = 0;
    int n = 0;

    while (size_read < size_to_read) {
      if (!blocking_client) {
        while (client_active.load()) {
          working_list = master_list;
          select_timeout.tv_sec = kTIMEOUT_S_NONBLOCK;
          select_timeout.tv_usec = kTIMEOUT_MS_NONBLOCK * 1000;

          n = select(sockfd + 1, &working_list, NULL, NULL, &select_timeout);

          if (n == -1) {
            std::cout << "ERROR " << strerror(errno) << ", on select at "
                      << __FILE__ << ":" << __LINE__ << std::endl;
            exit(0);
          }

          if (n == 1) break;
        }
      }

      n = ::read(sockfd, (char*)(&incoming_packet_size) + size_read,
                 size_to_read - size_read);

      if (n <= 0) {
        break;
      }

      size_read += n;
    }

    if (incoming_packet_size <= 0) {
      // disconnected
      client_active.store(false);
      continue;
    }

    cur_loc_mutex.lock();

    if (!got_first_msg) {
      got_first_msg = true;
    } else {
      cur_loc += last_packet_size;
      cur_loc = cur_loc % shm_size;
    }

    last_packet_size = incoming_packet_size;

    cur_loc_mutex.unlock();
  }

  return;
}

void SHMClient::shmRead(void* data, int data_size) {
  // std::cout << "client read  to " << cur_loc << std::endl;

  if (cur_loc + data_size > shm_size) {
    int part1 = shm_size - cur_loc;
    int part2 = data_size - part1;

    memcpy((char*)data, shm + cur_loc, part1);
    cur_loc = 0;
    memcpy((char*)data + part1, shm, part2);
    cur_loc += part2;
  } else {
    memcpy(data, shm + cur_loc, data_size);
    cur_loc += data_size;
  }
}

int SHMClient::read(void** pbuffer, timespec* ptms, char* data_type,
                    unsigned long* matrix_width, unsigned long* matrix_height,
                    unsigned int* matrix_type) {
  size_t size_of_msg = 0;

  if (!client_active.load()) return 0;

  // todo LOCK semaphore
  cur_loc_mutex.lock();
  unsigned int tmp_size = last_packet_size;

  if (last_packet_size > 0) {
    // data_type first
    bool free_data_type = (data_type == NULL);
    if (free_data_type) {
      data_type = (char*)malloc(sizeof(char));
    }

    // 1 - datatype
    shmRead(data_type, sizeof(*data_type));

    if (*data_type == kDATA_TYPE_MATRIX) {
      // 2/3/4 - width,height,type
      bool free_width = (matrix_width == NULL);
      if (free_width) {
        matrix_width = (unsigned long*)malloc(sizeof(matrix_width));
      }

      shmRead(matrix_width, sizeof(*matrix_width));
      if (free_width) {
        free(matrix_width);
        matrix_width = NULL;
      }

      bool free_height = (matrix_height == NULL);
      if (free_height) {
        matrix_height = (unsigned long*)malloc(sizeof(matrix_height));
      }

      shmRead(matrix_height, sizeof(*matrix_height));
      if (free_height) {
        free(matrix_height);
        matrix_height = NULL;
      }

      bool free_type = (matrix_type == NULL);
      if (free_type) {
        matrix_type = (unsigned int*)malloc(sizeof(matrix_type));
      }

      shmRead(matrix_type, sizeof(*matrix_type));
      if (free_type) {
        free(matrix_type);
        matrix_type = NULL;
      }
    }

    // std::cout << "last packet size = " << last_packet_size << std::endl;
    bool free_ptms = false;
    if (ptms == NULL) {
      free_ptms = true;
      ptms = (timespec*)malloc(sizeof(struct timespec));
    }

    shmRead(&(ptms->tv_sec), sizeof(ptms->tv_sec));
    shmRead(&(ptms->tv_nsec), sizeof(ptms->tv_nsec));

    // std::cout << "ptms sec: " << ptms->tv_sec <<  std::endl;
    // std::cout << "ptms nsec: " << ptms->tv_nsec <<  std::endl;

    if (free_ptms) free(ptms);

    shmRead(&size_of_msg, sizeof(size_of_msg));

    if (*pbuffer == NULL) *pbuffer = malloc(size_of_msg);
    shmRead(*pbuffer, size_of_msg);

    last_packet_size = 0;
  }

  cur_loc_mutex.unlock();
  // todo UNLOCK semaphore

  // std::cout << " read a message of size " << std::to_string(tmp_size)
  //           << std::endl;

  return size_of_msg;
}

SHMClient::~SHMClient() {
  client_active.store(false);
  incoming_message_thread.join();

  close(sockfd);

  if (shmdt(shm) == -1) {
    std::cout << "ERROR " << strerror(errno) << ", on shmdt at " << __FILE__
              << ":" << __LINE__ << std::endl;
    exit(0);
  }

  if (shmctl(shm_id, IPC_RMID, NULL) == -1) {
    std::cout << "ERROR " << strerror(errno) << ", on shmctl at " << __FILE__
              << ":" << __LINE__ << std::endl;
    if (errno != EINVAL)
      exit(0);
  }
}

SHMServer::SHMServer(std::string SHMName, unsigned long SHMSize,
                     unsigned short port_, bool blocking) {
  int enable = 1;
  port = port_;
  blocking_server = blocking;

  if (!blocking_server) {
    auto now = std::chrono::system_clock::now();
    auto now_c = std::chrono::system_clock::to_time_t(now);
    auto seconds = std::chrono::time_point_cast<std::chrono::seconds>(now);
    auto fraction = now - seconds;
    auto milliseconds =
        std::chrono::duration_cast<std::chrono::milliseconds>(fraction);

    std::cout << "[WARNING] BF NONBLOCK SERVER @"
              << std::put_time(std::localtime(&now_c), "%c")
              << std::to_string(milliseconds.count()) << "\n";
  }

  sockfd_packetsize = socket(AF_INET, SOCK_STREAM, 0);
  if (sockfd_packetsize < 0) {
    std::cout << "ERROR " << strerror(errno) << ", opening socket at "
              << __FILE__ << ":" << __LINE__ << std::endl;
    exit(0);
  }

  if (!blocking_server) {
    if (fcntl(sockfd_packetsize, F_SETFL, O_NONBLOCK) < 0) {
      std::cout << "ERROR " << strerror(errno)
                << ", setting socket to nonblock at " << __FILE__ << ":"
                << __LINE__ << std::endl;
      exit(0);
    }
  }

  if (setsockopt(sockfd_packetsize, SOL_SOCKET, SO_REUSEADDR, &enable,
                 sizeof(int)) < 0) {
    std::cout << "ERROR " << strerror(errno)
              << ", setting socket option (SO_REUSEADDR) at " << __FILE__ << ":"
              << __LINE__ << std::endl;
    exit(0);
  }

  bzero((char*)&serv_addr, sizeof(serv_addr));
  serv_addr.sin_family = AF_INET;
  serv_addr.sin_addr.s_addr = INADDR_ANY;
  serv_addr.sin_port = htons(port);
  if (bind(sockfd_packetsize, (struct sockaddr*)&serv_addr, sizeof(serv_addr)) <
      0) {
    std::cout << "ERROR " << strerror(errno) << ", on binding at " << __FILE__
              << ":" << __LINE__ << std::endl;
    exit(0);
  }

  socklen_t len = sizeof(serv_addr);
  if (getsockname(sockfd_packetsize, (struct sockaddr*)&serv_addr, &len) ==
      -1) {
    std::cout << "ERROR " << strerror(errno) << ", on getsockname at "
              << __FILE__ << ":" << __LINE__ << std::endl;
    exit(0);
  } else
    port = ntohs(serv_addr.sin_port);

  // figure out the host_name
  char hostname_[128];
  gethostname(hostname_, 128);
  host_name = std::string(hostname_);

  if (!blocking_server) {
    FD_ZERO(&connection_working_list);
    FD_ZERO(&connection_master_list);

    FD_SET(sockfd_packetsize, &connection_master_list);

    server_active.store(true);
    server_connected.store(false);
  }

  // make shared memory
  shm_size = SHMSize;
  shm_name = kSHM_PREFIX + SHMName + std::to_string(port) + ".shm";
  shm = NULL;

  std::ofstream shm_file(shm_name);
  shm_file.close();
  if ((shm_key = ftok(shm_name.c_str(), 'R')) == -1) {
    std::cout << "ERROR " << strerror(errno) << ", on ftok at " << __FILE__
              << ":" << __LINE__ << std::endl;
    exit(0);
  }

  if ((shm_id = shmget(shm_key, shm_size, 0644 | IPC_CREAT)) == -1) {
    std::cout << "ERROR " << strerror(errno) << ", on shmget at " << __FILE__
              << ":" << __LINE__ << std::endl;
    exit(0);
  }

  shm = (char*)shmat(shm_id, (void*)0, 0);
  if (shm == (char*)-1) {
    std::cout << "ERROR " << strerror(errno) << ", on shmat at " << __FILE__
              << ":" << __LINE__ << std::endl;
    exit(0);
  }

  shm_current_write = 0;

  // MAKE semaphore -- todo
}

bool SHMServer::listen_to_client(int retry_count,
                                 std::atomic_bool* notify_bool) {
  listen(sockfd_packetsize, 10);
  socklen_t clilen = sizeof(cli_addr);

  if (notify_bool) {
    notify_bool->store(true);
  }

  if (!blocking_server) {
    timeval select_timeout;
    select_timeout.tv_sec = kTIMEOUT_S_NONBLOCK;
    select_timeout.tv_usec = kTIMEOUT_MS_NONBLOCK * 1000;

    while (server_active.load()) {
      connection_working_list = connection_master_list;

      static int notification_counter = kNotificationWaitTime;
      notification_counter++;

      select_timeout.tv_sec = kTIMEOUT_S_NONBLOCK;
      select_timeout.tv_usec = kTIMEOUT_MS_NONBLOCK * 1000;

      int n = select(sockfd_packetsize + 1, &connection_working_list, NULL,
                     NULL, &select_timeout);

      if (notification_counter > kNotificationWaitTime) {
        std::cout << "Accepting connections @ " << std::to_string(port)
                  << std::endl;
        notification_counter = 0;
      }

      if (n == -1) {
        std::cout << "ERROR " << strerror(errno) << ", on select at "
                  << __FILE__ << ":" << __LINE__ << std::endl;
        exit(0);
      }

      if (n == 1) {
        break;
      }

      retry_count--;
      // time out -- return false and let the caller call this again
      if (retry_count == 0) return false;
    }
  }

  newsockfd_packetsize =
      accept(sockfd_packetsize, (struct sockaddr*)&cli_addr, &clilen);
  if (newsockfd_packetsize < 0) {
    // std::cout<<"ERROR on accept at "<<__FILE__<<":"<< __LINE__<<std::endl;
    return false;
  }

  if (!blocking_server) {
    if (fcntl(newsockfd_packetsize, F_SETFL, O_NONBLOCK) < 0) {
      std::cout << "ERROR " << strerror(errno)
                << ", setting socket to nonblock at " << __FILE__ << ":"
                << __LINE__ << std::endl;
      exit(0);
    }

    FD_ZERO(&data_working_list);
    FD_ZERO(&data_master_list);

    FD_SET(newsockfd_packetsize, &data_master_list);
    server_connected.store(true);
  }

  return true;
}

int SHMServer::shmWrite(const void* buffer, size_t buffer_size) {
  // std::cout << "server writing to " << shm_current_write << std::endl;
  if (shm_current_write + buffer_size > shm_size) {
    long part1 = shm_size - shm_current_write;
    long part2 = buffer_size - part1;

    memcpy(shm + shm_current_write, (char*)buffer, part1);
    shm_current_write = 0;
    memcpy(shm + shm_current_write, (char*)buffer + part1, part2);
    shm_current_write += part2;
  } else {
    memcpy(shm + shm_current_write, (char*)buffer, buffer_size);
    shm_current_write += buffer_size;
  }

  return buffer_size;
}

int SHMServer::write(const void* buffer, size_t size, timespec* ptms,
                     unsigned long width, unsigned long height,
                     unsigned int matrix_type) {
  int n;
  bool free_ptms = false;

  int total_packet_size = size + sizeof(size) + sizeof(timespec) +
                          sizeof(unsigned long) + sizeof(unsigned long) +
                          sizeof(unsigned int) + sizeof(char);

  if (total_packet_size > shm_size) {
    std::cout << "ERROR packet size (" << std::to_string(total_packet_size)
              << ") is bigger than Shared Memory space ("
              << std::to_string(shm_size)
              << "). Make sure you setup your SHM sizes correctly."
              << std::endl;
    return -1;
  }

  if (ptms == NULL) {
    free_ptms = true;
    ptms = (timespec*)malloc(sizeof(timespec));
    if (get_time(ptms)) {
      std::cout << "ERROR get_time";
      exit(0);
    }
  }

  // todo LOCK semaphore
  char data_type = kDATA_TYPE_DATA;
  if (width != 0 || height != 0 || matrix_type != 0) {
    data_type = kDATA_TYPE_MATRIX;
  } else {
    total_packet_size -=
        sizeof(unsigned long) + sizeof(unsigned long) + sizeof(unsigned int);
  }

  shmWrite(&data_type, sizeof(data_type));

  if (data_type == kDATA_TYPE_MATRIX) {
    shmWrite(&width, sizeof(width));
    shmWrite(&height, sizeof(height));
    shmWrite(&matrix_type, sizeof(matrix_type));
  }

  shmWrite(ptms, sizeof(timespec));
  shmWrite(&size, sizeof(size_t));
  shmWrite(buffer, size);

  if (free_ptms) {
    free(ptms);
    ptms = NULL;
  }

  // todo UNLOCK semaphore

  timeval select_timeout;
  select_timeout.tv_sec = kTIMEOUT_S_NONBLOCK;
  select_timeout.tv_usec = kTIMEOUT_MS_NONBLOCK * 1000;  // 500ms

  int size_to_write = 0;
  int size_written = 0;

  size_to_write = sizeof(total_packet_size);

  while (size_written < size_to_write) {
    if (!blocking_server) {
      while (server_connected.load()) {
        data_working_list = data_master_list;
        select_timeout.tv_sec = kTIMEOUT_S_NONBLOCK;
        select_timeout.tv_usec = kTIMEOUT_MS_NONBLOCK * 1000;

        n = select(newsockfd_packetsize + 1, NULL, &data_working_list, NULL,
                   &select_timeout);

        if (n == -1) {
          std::cout << "ERROR " << strerror(errno) << ", on select at "
                    << __FILE__ << ":" << __LINE__ << std::endl;
          exit(0);
        }

        if (n == 1) break;
      }
    }

    n = ::write(newsockfd_packetsize,
                (char*)(&total_packet_size) + size_written,
                size_to_write - size_written);

    if (n <= 0) {
      break;
    }

    size_written += n;
  }

  if (n < 0) {
    std::cout << "ERROR " << strerror(errno) << ", writing to socket at "
              << __FILE__ << ":" << __LINE__ << std::endl;
    ;
    return n;
  }

  // std::cout << " wrote packet of size " << std::to_string(total_packet_size)
  //           << std::endl;

  return size;
}

void SHMServer::close_client(bool notify_client) {
  server_connected.store(false);

  if (notify_client) {
    int termination_message = -1;

    timeval select_timeout;
    select_timeout.tv_sec = kTIMEOUT_S_NONBLOCK;
    select_timeout.tv_usec = kTIMEOUT_MS_NONBLOCK * 1000;  // 500ms

    int size_to_write = 0;
    int size_written = 0;
    int n = 0;

    size_to_write = sizeof(termination_message);

    while (size_written < size_to_write) {
      if (!blocking_server) {
        while (server_connected.load()) {
          data_working_list = data_master_list;
          select_timeout.tv_sec = kTIMEOUT_S_NONBLOCK;
          select_timeout.tv_usec = kTIMEOUT_MS_NONBLOCK * 1000;

          n = select(newsockfd_packetsize + 1, NULL, &data_working_list, NULL,
                     &select_timeout);

          if (n == -1) {
            std::cout << "ERROR " << strerror(errno) << ", on select at "
                      << __FILE__ << ":" << __LINE__ << std::endl;
            exit(0);
          }

          if (n == 1) break;
        }
      }

      n = ::write(newsockfd_packetsize,
                  ((char*)&termination_message) + size_written,
                  size_to_write - size_written);

      if (n <= 0) {
        break;
      }

      size_written += n;
    }
  }

  close(newsockfd_packetsize);

  // delete shared memory
  if (shmdt(shm) == -1) {
    std::cout << "ERROR " << strerror(errno) << ", on shmdt at " << __FILE__
              << ":" << __LINE__ << std::endl;
    exit(0);
  }

  if (shmctl(shm_id, IPC_RMID, NULL) == -1) {
    std::cout << "ERROR " << strerror(errno) << ", on shmctl at " << __FILE__
              << ":" << __LINE__ << std::endl;
    if ( errno != EINVAL )
      exit(0);
  }

  shm_id = 0;
  shm = NULL;

  // todo DELETE semaphore
}

SHMServer::~SHMServer() {
  server_active.store(false);

  if (shm != NULL) {
    std::cout << "ERROR, SHM is not NULL at " << __FILE__ << ":" << __LINE__
              << std::endl;
  }

  close(sockfd_packetsize);
}

}  // namespace bf
