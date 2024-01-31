// if logs from vision should be printed to stdout
#define LOGGING_ENABLED true
// if you are on windows and are simulating robot code
#define SIMULATION_ON_WINDOWS false

#if SIMULATION_ON_WINDOWS
#pragma comment(lib, "Ws2_32.lib")
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <sys/types.h>
#include <unistd.h>
#endif
#include <chrono>
#include <exception>
#include <iostream>
#include <regex>
#include <chrono>
#include <string.h>
#include <thread>

#include "Util/SocketClient.h"

#define SOCK_CLIENT_BUF_SIZE 1024

const std::string regexp = R"(\^([-+]?[0-9]*\.?[0-9]+([eE][-+]?[0-9]+)?),([-+]?[0-9]*\.?[0-9]+([eE][-+]?[0-9]+)?),([-+]?[0-9]*\.?[0-9]+([eE][-+]?[0-9]+)?),([-+]?[0-9]*\.?[0-9]+([eE][-+]?[0-9]+)?),([-+]?[0-9]*\.?[0-9]+([eE][-+]?[0-9]+)?),([-+]?[0-9]*\.?[0-9]+([eE][-+]?[0-9]+)?),([-+]?[0-9]*\.?[0-9]+([eE][-+]?[0-9]+)?)\$$)";

#define GET_CUR_TIME_MS \
  std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::system_clock::now().time_since_epoch()).count();

#if SIMULATION_ON_WINDOWS
#define close_socket(sock) \
  closesocket(sock);       \
  WSACleanup();
#else
#define close_socket(sock) close(sock);
#endif

/**
 * Constructor
 *
 *
 * @param host The hostname string of jetson
 * @param port The port of socket server with jetson
 * @param staleTime The amount of time, in ms, after the rio gets the data from the jetson before data is considered stale.
 * @param deadTime The amount of time, in ms, after the rio gets the data from the jetson before the jetson is considered dead (and it will retry connection);
 * This value should be greater than staleTime.
 */
SocketClient::SocketClient(std::string host, int port, unsigned long long staleTime, unsigned long long deadTime)
    : m_host{host}, m_port{port}, m_staleTime{staleTime}, m_deadTime{deadTime} {}

/**
 * Initializes thread that fetches data from socket server
 *
 * @warning Only call this once!!
 */
void SocketClient::Init()
{
  m_th = std::thread([this]
                     { this->m_SocketLoop(m_host, m_port); });
  m_th.detach();
}

/**
 * Debug thread logging
 * https://stackoverflow.com/questions/2342162/stdstring-formatting-like-sprintf
 */
template <typename... Args>
void SocketClient::Log(const char *format, Args... args)
{
#if LOGGING_ENABLED
  // behave like printf
  int size_s = std::snprintf(nullptr, 0, format, args...) + 1;
  if (size_s <= 0)
  {
    throw std::runtime_error("Error during formatting.");
  }
  auto size = static_cast<size_t>(size_s);
  std::unique_ptr<char[]> buf(new char[size]);
  std::snprintf(buf.get(), size, format, args...);
  std::string str = std::string(buf.get(), buf.get() + size - 1);
  int curr_time = GET_CUR_TIME_MS;
  m_logQueue.push("[" + std::to_string(curr_time) + "] " + str);
#endif
}

/**
 * Returns true if connection has been established with jetson
 *
 * @note if the websocket connects then the jetson dies, then this will still return true.
 * If that is the case, use IsStale() to check the data instead.
 *
 * @returns If connection is established.
 */
bool SocketClient::HasConn()
{
  return m_hasConn.load();
}

/**
 * Returns if age of data is too long, determined by the last time the rio has gotten data from the jetson
 *
 * Use this to check if the jetson is dead; need to manually determine staleness from age of data given in vector.
 *
 * @warning If this is true, do not trust GetData().
 *
 * @returns If data is stale
 */
bool SocketClient::IsStale()
{
  unsigned long long curTimeMs = GET_CUR_TIME_MS;
  double lastTime = m_lastTimeMs.load();
  // bool hasInit = m_hasInit.load();

  return (curTimeMs - lastTime >= m_staleTime);
}

/**
 * Gets the data in vector form.
 *
 * The vector is [camId, tagId, x, y, angZ, age, uniqueId]
 *
 * @warning Do not trust this method if IsStale() is true.
 *
 * @returns Data in vector form
 */
std::vector<double> SocketClient::GetData()
{
  double camId = m_camId.load();
  double tagId = m_tagId.load();
  double x = m_x.load();
  double y = m_y.load();
  double z = m_angZ.load();
  double age = m_age.load();
  double uniqueId = m_count.load();

  return std::vector<double>{camId, tagId, x, y, z, age, uniqueId};
}

/**
 * The loop that runs the socket
 */
void SocketClient::m_SocketLoop(std::string host, int port)
{
  int sockfd;

  // get the host info from params
  struct addrinfo hints = {}, *addrs;
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_protocol = IPPROTO_TCP;

  int status = getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &addrs);
  while (status != 0)
  {
    // std::cout << "getaddrinfo fail " << status << "\n";
    Log("getaddrinfo fail %s", gai_strerror(status));

    // try again until good
    status = getaddrinfo(host.c_str(), std::to_string(port).c_str(), &hints, &addrs);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }
  // std::cout << "getaddrinfo success\n";
  Log("getaddrinfo success");
  for (struct addrinfo *addr = addrs; addr != NULL; addr = addr->ai_next)
  {
    // std::cout << "addr->ai_addr: " << addr->ai_addr << "\n";
    Log("addr->ai_addr: %s", inet_ntoa(((struct sockaddr_in *)addr->ai_addr)->sin_addr));
  }
  // std::cout << "length of addrs: " << sizeof(addrs) << "\n";
  // std::cout << std::endl;
  Log("length of addrs: %d", sizeof(addrs));
  Log("");

  // socket create and verification
  sockfd = socket(addrs->ai_family, addrs->ai_socktype, addrs->ai_protocol);
  while (sockfd == -1)
  {
    // try again until good
    sockfd = socket(addrs->ai_family, addrs->ai_socktype, addrs->ai_protocol);
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  // std::cout << "Good socket creation\n";
  Log("Good socket creation");

  // connect the client socket to server socket
  int res = connect(sockfd, addrs->ai_addr, addrs->ai_addrlen);
  while (res != 0)
  {
    // printf("connection with the server failed ...\n");
    Log("initial connection with the server failed because %s", strerror(errno));

    close_socket(sockfd); // destroy socket
    std::this_thread::sleep_for(std::chrono::seconds(1));

    // after 1 second delay, try init socket again
    // socket create and verification
    sockfd = socket(addrs->ai_family, addrs->ai_socktype, addrs->ai_protocol);
    while (sockfd == -1)
    {
      // try again until good
      sockfd = socket(addrs->ai_family, addrs->ai_socktype, addrs->ai_protocol);
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

    // std::cout << "Good socket creation\n";
    Log("Good socket creation");

    // connect the client socket to server socket
    res = connect(sockfd, addrs->ai_addr, addrs->ai_addrlen);
  }

  m_hasConn.store(true);
  m_camId.store(0);
  m_tagId.store(0);
  m_x.store(0);
  m_y.store(0);
  m_angZ.store(0);
  m_age.store(0);
  m_count.store(0);
  m_hasInit.store(false);

  while (true)
  {
    // std::this_thread::sleep_for(std::chrono::milliseconds(1000));
    unsigned long long curTimeMs = GET_CUR_TIME_MS;
    unsigned long long lastTimeMs = m_lastTimeMs.load();

    if (m_hasInit && curTimeMs - lastTimeMs >= m_deadTime)
    {
      // attempts to reconnect if rpi dies mid-match
      res = -1;
      while (res != 0)
      {
        // std::cout << "connection with the server failed ...\n";
        Log("connection with the server failed ...");

        close_socket(sockfd); // destroy socket
        std::this_thread::sleep_for(std::chrono::seconds(1));

        // after 1 second delay, try everything
        // socket create and verification
        sockfd = socket(addrs->ai_family, addrs->ai_socktype, addrs->ai_protocol);
        while (sockfd == -1)
        {
          // try again until good
          sockfd = socket(addrs->ai_family, addrs->ai_socktype, addrs->ai_protocol);
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }

        // std::cout << "Good socket creation\n";
        Log("Good socket creation");

        // connect the client socket to server socket
        res = connect(sockfd, addrs->ai_addr, addrs->ai_addrlen);
      }
    }

    char buff[SOCK_CLIENT_BUF_SIZE];
    memset(buff, 0, sizeof(buff));
#if SIMULATION_ON_WINDOWS
    recv(sockfd, buff, sizeof(buff), 0);
#else
    read(sockfd, buff, sizeof(buff));
#endif
    buff[SOCK_CLIENT_BUF_SIZE - 1] = '\0';

    std::regex exp(regexp);
    std::string inp(buff);

    // std::cout << "inp: " << inp << std::endl;
    Log("inp: %s", inp);
    if (inp[0] == '0')
    {
      // store time
      m_hasInit.store(true);
      m_lastTimeMs.store(curTimeMs);
      continue;
    }

    // filter string until last dollar sign, where regex would register
    int idx = inp.size();
    for (int i = inp.size() - 1; i >= 0; i--)
    {
      if (inp[i] == '$')
      {
        idx = i + 1;
        break;
      }
    }

    inp = inp.substr(0, idx);

    std::smatch matches;
    auto res = std::regex_match(inp, matches, exp);
    if (res)
    {
      // parses regex
      std::string sCamId, sTagId, sX, sY, sAngZ, sAge, sCount;

      sCamId = matches[1];
      sTagId = matches[3];
      sX = matches[5];
      sY = matches[7];
      sAngZ = matches[9];
      sAge = matches[11];
      sCount = matches[13];

      // update data
      m_camId.store(std::stod(sCamId));
      m_tagId.store(std::stod(sTagId));
      m_x.store(std::stod(sX));
      m_y.store(std::stod(sY));
      m_angZ.store(std::stod(sAngZ));
      m_age.store(std::stod(sAge));
      m_count.store(std::stod(sCount));

      // store time
      m_lastTimeMs.store(curTimeMs);
    }
  }
}