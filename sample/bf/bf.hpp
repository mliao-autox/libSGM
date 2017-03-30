/*
 * ----------------------------------------------------------------------------
 * Binary Flow (bf): A Minimalist IPC middleware
 * Copyright (C) 2016 AutoX, Inc.
 * ----------------------------------------------------------------------------
 */

#ifndef BF_HPP
#define BF_HPP

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
#include <iomanip>
#include <iostream>
#include <mutex>
#include <thread>
#include <vector>
#ifdef __MACH__
#include <mach/clock.h>
#include <mach/mach.h>
#endif
#include <sys/ipc.h>
#include <sys/shm.h>
#include <atomic>

namespace bf {

#define kDATA_TYPE_DATA ((char)'d')
#define kDATA_TYPE_MATRIX ((char)'m')

const int kNotificationWaitTime = 2000;

bool get_time(timespec* ptms);

void print_time(timespec ptms);

timespec subtract_time(timespec a, timespec b);

bool a_eq_later_than_b(timespec a, timespec b);

bool a_same_time_with_b(timespec a, timespec b);

class Client {
  int sockfd;
  bool blocking_client;

  fd_set working_list;
  fd_set master_list;
  std::atomic_bool client_active;

 public:
  Client(std::string host_name, unsigned short host_port, bool blocking = false,
         bool* pResult = NULL);
  int read(void** pbuffer, timespec* ptms = NULL, char* data_type = NULL,
           unsigned long* matrix_width = NULL,
           unsigned long* matrix_height = NULL,
           unsigned int* matrix_elemSize = NULL);
  int write(const void* buffer, size_t size, timespec* ptms = NULL);
  ~Client();
};

class Server {
  int sockfd;
  int newsockfd;
  struct sockaddr_in serv_addr;
  struct sockaddr_in cli_addr;
  bool blocking_server;

  fd_set connection_working_list;
  fd_set connection_master_list;
  std::atomic_bool server_active;

  fd_set data_working_list;
  fd_set data_master_list;
  std::atomic_bool server_connected;

 public:
  std::string host_name;
  unsigned short port;
  Server(unsigned short port_ = 0, bool blocking = false);
  bool listen_to_client(
      int retry_count = 1,
      std::atomic_bool* notify_bool =
          NULL);  // retry_count is only valid for nonblocking servers
  int read(void** pbuffer, timespec* ptms = NULL);
  int write(const void* buffer, size_t size, timespec* ptms = NULL,
            unsigned long width = 0, unsigned long height = 0,
            unsigned int elemSize = 0);
  void close_client();
  ~Server();
};

class SHMClient {
  int sockfd;
  bool blocking_client;

  fd_set working_list;
  fd_set master_list;
  
  std::thread incoming_message_thread;

  key_t shm_key;
  int shm_id;
  unsigned long shm_size;
  char* shm;
  bool got_first_msg;
  unsigned long cur_loc;
  unsigned long last_packet_size;

  std::mutex cur_loc_mutex;
  std::atomic_bool client_active;

  void check_for_incoming_message();
  void shmRead(void* data, int data_size);

 public:
  bool isActive();
  SHMClient(std::string host_name, unsigned short host_port,
            std::string SHMname, unsigned long SHMsize, bool blocking = false,
            bool* pResult = NULL);
  int read(void** pbuffer, timespec* ptms = NULL, char* data_type = NULL,
           unsigned long* matrix_width = NULL,
           unsigned long* matrix_height = NULL,
           unsigned int* matrix_elemSize = NULL);
  // no longer needed
  // int write(const void* buffer, size_t size, timespec* ptms = NULL);
  ~SHMClient();
};

class SHMServer {
  int sockfd_packetsize;
  int newsockfd_packetsize;
  struct sockaddr_in serv_addr;
  struct sockaddr_in cli_addr;
  bool blocking_server;

  key_t shm_key;
  int shm_id;
  unsigned long shm_size;
  char* shm;
  unsigned long shm_current_write;

  fd_set connection_working_list;
  fd_set connection_master_list;
  std::atomic_bool server_active;

  fd_set data_working_list;
  fd_set data_master_list;
  std::atomic_bool server_connected;

  int shmWrite(const void* buffer, size_t buffer_size);

 public:
  std::string host_name;
  std::string shm_name;
  unsigned short port;
  SHMServer(std::string SHMName, unsigned long SHMSize = 1000000,
            unsigned short port_ = 0, bool blocking = false);
  bool listen_to_client(
      int retry_count = 1,
      std::atomic_bool* notify_bool =
          NULL);  // retry_count is only valid for nonblocking servers
  // this is no longer needed, maybe in future
  // int read(void** pbuffer, timespec* ptms = NULL);
  int write(const void* buffer, size_t size, timespec* ptms = NULL,
            unsigned long width = 0, unsigned long height = 0,
            unsigned int elemSize = 0);
  void close_client(bool notify_client = false);
  ~SHMServer();
};

bool rpc(std::string host_name, unsigned short host_port,
         const void* send_msg_ptr, size_t send_msg_len, void** receive_msg_buf,
         int* receive_msg_buf_size);
bool nonblock_rpc(std::string host_name, unsigned short host_port,
                  const void* send_msg_ptr, size_t send_msg_len,
                  void** receive_msg_buf, int* receive_msg_buf_size);

void run_master(unsigned short port = 11411);

struct Datum {
  char data_type;
  timespec time;
  void* data;
  size_t size;
  unsigned long width;
  unsigned long height;
  unsigned int matrix_type;
};

class Publisher {
  std::thread listen_thread;
  std::future<bool> listen_return;
  std::mutex mu;
  void lock();
  void unlock();
  Server* subscription_server;
  std::vector<Server*> data_servers;
  std::vector<SHMServer*> shm_data_servers;
  std::atomic_bool actively_listening;
  std::atomic_bool ready_to_notify_subscriber;

  unsigned long shm_size;
  bool shm_publisher;

 public:
  std::string publisher_name;
  void listen();
  Publisher(std::string publisher_name_, std::string master_name = "master",
            unsigned long _shm_size = 25000000,
            unsigned short master_port = 11411, bool listen_blocking = false);
  void publish(void* buffer, unsigned long width, unsigned long height,
               unsigned int elemSize, unsigned int matrix_type, timespec* ptms = NULL);
  void publish(void* buffer, size_t size, timespec* ptms = NULL);
  void publish(char* buffer, size_t size, timespec* ptms = NULL);
  void publish(double* buffer, size_t size, timespec* ptms = NULL);
  void publish(int* buffer, size_t size, timespec* ptms = NULL);
  void publish(float* buffer, size_t size, timespec* ptms = NULL);
  void publish(bool* buffer, size_t size, timespec* ptms = NULL);
  ~Publisher();
};

class Subscriber {
  std::string master_name;
  unsigned short master_port;
  bool initialized;
  void initialize();

  std::atomic_bool actively_listening;

  std::thread listen_thread;
  std::mutex mu;
  Client* client;
  SHMClient* shmClient;

 public:
  std::string publisher_name;
  std::vector<Datum> buffer;
  void lock();
  void unlock();
  Datum get_the_lastest_and_clear_all();
  Datum pop_first();
  Datum pop_last();
  Datum sync_read();
  void listen();
  Subscriber(std::string publisher_name, std::string master_name = "master",
             unsigned short master_port = 11411);
  ~Subscriber();
};

}  // namespace bf

#endif
