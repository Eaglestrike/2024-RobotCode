#pragma once

#include <sys/stat.h>
#include <sys/types.h>
#include <unistd.h>

#include <algorithm>
#include <chrono>
#include <ctime>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <map>
#include <queue>
#include <set>
#include <sstream>
#include <string>
#include <vector>

/**
 * Log levels
 */
enum class LogLevel : int {
  TRACE = 0,
  DEBUG = 1,
  INFO = 2,
  WARN = 3,
  ERROR = 4,
  FATAL = 5,
  OFF = 6
};

namespace {
/**
 * Time manager class
 */
class TimeMgr {
public:
  /**
   * Gets instance of time manager
   *
   * @returns time manager
   */
  static TimeMgr &GetInstance() {
    static TimeMgr timemgr;
    return timemgr;
  }

  TimeMgr(TimeMgr const &) = delete;
  void operator=(TimeMgr const &) = delete;

  /**
   * Double to string
   *
   * @param val Value
   * @param precision Precision
   */
  static std::string DoubleToString(double val, int precision) {
    std::stringstream stream;
    stream << std::fixed << std::setprecision(precision) << val;
    return stream.str();
  }

  /**
   * Gets current time str. If no custom time then gets system time
   *
   * @returns Current time str
   */
  std::string GetCurTimeSStr() const {
    if (m_curTime < 0) {
      return DoubleToString(GetCurTimeS(), 6);
    }

    return DoubleToString(m_curTime, 6);
  }

  /**
   * Gets current system time in date time form
   *
   * @returns Current system time in datetime form
   */
  static std::string GetTimeStr() {
    const auto now = std::chrono::system_clock::now();
    std::time_t now_c = std::chrono::system_clock::to_time_t(now);
    std::tm *tm = std::localtime(&now_c);
    if (!tm) {
      return "time_error";
    }

    char dateStr[100];
    std::strftime(dateStr, sizeof(dateStr), "%c", tm);

    std::string ret = dateStr;
    return ret;
  }

  /**
   * Updates current time, to sync with clock. Once this is updated with
   * external time it will always rely on the latest external time.
   *
   * @param time current time in s, if negative then uses std::chrono time
   */
  void UpdateTime(double time) {
    if (time < 0) {
      m_curTime = GetCurTimeS();

      return;
    }

    m_curTime = time;
  }

private:
  /**
   * Constructor
   */
  TimeMgr() : m_curTime{-1} {}

  double m_curTime;

  /**
   * Gets current system time in seconds
   *
   * @returns system time in seconds
   */
  static double GetCurTimeS() {
    const auto now = std::chrono::system_clock::now().time_since_epoch();
    return std::chrono::duration_cast<std::chrono::seconds>(now).count();
  }
};

/**
 * File logger for general logging
 */
class FileLogger {
public:
  /**
   * Constructor
   *
   * @param fname File  name
   * @param name Logger name
   */
  FileLogger(std::string fname, std::string name)
      : m_fname{fname}, m_name{name} {}

  /**
   * Enables logger
   */
  void Enable() {
    if (m_fout.is_open()) {
      return;
    }

    std::queue<std::string> empty;
    std::swap(m_toLog, empty);

    m_fout.open(m_fname, std::ios::app);
  }

  /**
   * Disabled logger
   */
  void Disable() {
    if (!m_fout.is_open()) {
      return;
    }

    m_fout.flush();
    m_fout.close();
  }

  /**
   * Sets level
   *
   * @param level Level
   */
  void SetLevel(LogLevel level) { m_level = level; }

  /**
   * Sets if should log to console
   *
   * @param logToConsole true if should, false if not
   */
  void SetLogToConsole(bool logToConsole) { m_logToConsole = logToConsole; }

  /**
   * Gets if should log to console
   *
   * @returns if should log to console
   */
  bool GetLogToConsole() const { return m_logToConsole; }

  /**
   * If enabled
   *
   * @returns If logger is enabled
   */
  bool IsEnabled() const { return m_fout.is_open(); }

  /**
   * Gets current level
   *
   * @returns current level
   */
  LogLevel GetLevel() const { return m_level; }

  /**
   * Logs trace
   *
   * @param heading Heading/title, to categorize log
   * @param msg Log message
   */
  void Trace(std::string heading, std::string msg) {
    LogLevel(heading, msg, LogLevel::TRACE);
  }

  /**
   * Logs debug
   *
   * @param heading Heading/title, to categorize log
   * @param msg Log message
   */
  void Debug(std::string heading, std::string msg) {
    LogLevel(heading, msg, LogLevel::DEBUG);
  }

  /**
   * Logs info
   *
   * @param heading Heading/title, to categorize log
   * @param msg Log message
   */
  void Info(std::string heading, std::string msg) {
    LogLevel(heading, msg, LogLevel::INFO);
  }

  /**
   * Logs warn
   *
   * @param heading Heading/title, to categorize log
   * @param msg Log message
   */
  void Warn(std::string heading, std::string msg) {
    LogLevel(heading, msg, LogLevel::WARN);
  }

  /**
   * Logs error
   *
   * @param heading Heading/title, to categorize log
   * @param msg Log message
   */
  void Error(std::string heading, std::string msg) {
    LogLevel(heading, msg, LogLevel::ERROR);
  }

  /**
   * Logs fatal
   *
   * @param heading Heading/title, to categorize log
   * @param msg Log message
   */
  void Fatal(std::string heading, std::string msg) {
    LogLevel(heading, msg, LogLevel::FATAL);
  }

  /**
   * Initializes File logger
   *
   * Creates file.
   */
  void Init() {
    Enable();
    Disable();
  }

  /**
   * Periodic call
   */
  void Periodic() {
    if (!IsEnabled()) {
      return;
    }

    WriteQueue();
  }

private:
  std::string m_fname;
  std::ofstream m_fout;

  std::string m_name;
  bool m_logToConsole;

  LogLevel m_level;

  std::queue<std::string> m_toLog;

  /**
   * Logs level
   *
   * @param heading Heading/title, to categorize log
   * @param msg Log message
   * @param level Log level
   */
  void LogLevel(std::string heading, std::string msg, LogLevel level) {
    if (!IsEnabled() || level < m_level) {
      return;
    }

    std::string logStr =
        m_name + " [" + GetLogLevelStr(level) + "] — " + heading + ": " + msg;

    m_toLog.push(logStr);
  }

  /**
   * Gets log level string
   *
   * @returns log level string
   */
  std::string GetLogLevelStr(enum LogLevel level) const {
    switch (level) {
    case LogLevel::TRACE:
      return "TRACE";
    case LogLevel::DEBUG:
      return "DEBUG";
    case LogLevel::INFO:
      return "INFO";
    case LogLevel::WARN:
      return "WARN";
    case LogLevel::ERROR:
      return "ERROR";
    case LogLevel::FATAL:
      return "FATAL";
    default:
      return "";
    }
  }

  /**
   * Writes everything in queue
   */
  void WriteQueue() {
    if (!IsEnabled() || m_toLog.empty()) {
      return;
    }

    while (!m_toLog.empty()) {
      std::string logStr = m_toLog.front();
      m_toLog.pop();

      if (m_logToConsole) {
        std::clog << TimeMgr::GetInstance().GetCurTimeSStr() << " — " << logStr
                  << "\n";
      }

      m_fout << TimeMgr::GetInstance().GetCurTimeSStr() << " — " << logStr
             << "\n";
    }
  }
};

/**
 * CSV logger that writes to file
 */
class CSVLogger {
public:
  /**
   * Constructor
   *
   * @param fname File name
   * @param headings Vector of headings
   *
   * @note in headings, do not use "__time__" or an empty string, no
   * repeat names. If you do this, logger won't log
   */
  CSVLogger(std::string fname, std::vector<std::string> headings)
      : m_fname{fname}, m_headings{headings}, m_headingSrch{m_headings.begin(),
                                                            m_headings.end()} {}

  /**
   * Enables logger by opening file
   */
  void Enable() {
    if (m_fout.is_open() || !IsValidConfiguration()) {
      return;
    }

    m_fout.open(m_fname, std::ios::app);
  }

  /**
   * Enables logger by closing file
   *
   * This must be done to copy file safely
   */
  void Disable() {
    if (!m_fout.is_open() || !IsValidConfiguration()) {
      return;
    }

    m_fout.flush();
    m_fout.close();
  }

  /**
   * If is enabled
   *
   * @returns If enabled
   */
  bool IsEnabled() const { return m_fout.is_open(); }

  /**
   * Logs string
   *
   * @param key Key to log, must be in headings vector passed in constructor
   * @param val Value to log, dont use underscores before and after name
   */
  void LogStr(std::string key, std::string val) {
    if (IsBadKey(key) || !IsValidConfiguration()) {
      return;
    }

    m_logVals[key] = val;
  }

  /**
   * Logs double to 4 decimal points
   *
   * @param key Key to log, must be in headings vector passed in constructor
   * @param val Value to log
   */
  void LogNum(std::string key, double val) { LogNum(key, val, 4); }

  /**
   * Logs double to specified precision
   *
   * @param key Key to log, must be in headings vector passed in constructor
   * @param val Value to log
   * @param precision Precision
   */
  void LogNum(std::string key, double val, int precision) {
    if (IsBadKey(key) || !IsValidConfiguration()) {
      return;
    }

    m_logVals[key] = TimeMgr::DoubleToString(val, precision);
  }

  /**
   * Logs boolean
   *
   * @param key Key to log, must be in headings vector passed in constructor
   * @param val Value to log
   */
  void LogBool(std::string key, bool val) {
    if (IsBadKey(key) || !IsValidConfiguration()) {
      return;
    }

    std::string s = val ? "TRUE" : "FALSE";
    m_logVals[key] = s;
  }

  /**
   * Logs a row of values
   *
   * @note size of row must be same as heading vector size
   *
   * @param row The row of values
   */
  void LogRow(std::vector<std::string> row) {
    if (row.size() != m_headings.size() || !IsValidConfiguration()) {
      return;
    }

    for (unsigned int i = 0; i < row.size(); i++) {
      std::string key = m_headings[i];
      m_logVals[key] = row[i];
    }
  }

  /**
   * Initializes CSV logger
   *
   * @note does not enable logging
   */
  void Init() {
    if (!IsValidConfiguration()) {
      return;
    }

    m_fout.open(m_fname);
    WriteHeadings();
    Disable();
  }

  /**
   * Periodic call
   */
  void Periodic() {
    if (!IsValidConfiguration() || !IsEnabled()) {
      return;
    }

    WriteMap();
  }

private:
  /**
   * Determines if key is bad
   *
   * @param key Key to determine if bad
   */
  bool IsBadKey(std::string key) const {
    return m_headingSrch.find(key) == m_headingSrch.end();
  }

  /**
   * Determines if valid configuration
   *
   * @returns If valid configuration
   */
  bool IsValidConfiguration() const {
    return m_headings.size() == m_headingSrch.size() &&
           m_headingSrch.find("__time__") == m_headingSrch.end();
  }

  /**
   * Writes headings
   */
  void WriteHeadings() {
    if (!IsEnabled()) {
      return;
    }

    m_fout << "__time__";
    for (const std::string &s : m_headings) {
      m_fout << "," << s;
    }
    m_fout << "\n";
  }

  /**
   * Writes map
   */
  void WriteMap() {
    if (!IsEnabled() || m_logVals.empty()) {
      return;
    }

    m_fout << TimeMgr::GetInstance().GetCurTimeSStr();

    for (const std::string &s : m_headings) {
      m_fout << "," << m_logVals[s];
    }

    m_fout << "\n";
  }

  std::string m_fname;
  std::ofstream m_fout;

  std::vector<std::string> m_headings;
  std::map<std::string, std::string> m_logVals;
  std::set<std::string> m_headingSrch;
};
} // namespace

/**
 * FRC Logger class
 */
class FRCLogger {
public:
  /**
   * Constructor
   *
   * @param name Logger Name
   * @param headings Vector of Headings
   *
   * @note in headings, do not use "__time__" or an empty string, no
   * repeat names. If you do this, logger won't log
   */
  FRCLogger(std::string name, std::vector<std::string> headings)
      : m_csv{FOLDER + m_dateStr + ".csv", headings}, m_file{FOLDER +
                                                                 m_dateStr +
                                                                 ".log",
                                                             name} {}

  /**
   * Init function
   */
  void Init() {
    if (m_initialized) {
      return;
    }

    m_initialized = true;

    // make directory if doesn't exist
    if (stat(FOLDER.c_str(), &m_st) == -1) {
      mkdir(FOLDER.c_str(), 0700);
    }

    m_csv.Init();
    m_file.Init();
  }

  /**
   * Periodic function
   *
   * Make sure to call this at END of periodic
   *
   * @param time Current time to sync up time, use seconds
   */
  void Periodic(double time) {
    TimeMgr::GetInstance().UpdateTime(time);

    m_csv.Periodic();
    m_file.Periodic();
  }

  /**
   * Enables logger by opening file
   */
  void Enable() {
    m_csv.Enable();
    m_file.Enable();
  }

  /**
   * Enables logger by closing file
   *
   * This must be done to copy file safely
   */
  void Disable() {
    m_csv.Disable();
    m_file.Disable();
  }

  /**
   * If is enabled
   *
   * @returns If enabled
   */
  bool IsEnabled() const { return m_csv.IsEnabled() && m_file.IsEnabled(); }

  /**
   * Logs string
   *
   * @param key Key to log, must be in headings vector passed in constructor
   * @param val Value to log, dont use underscores before and after name
   */
  void LogStr(std::string key, std::string val) { m_csv.LogStr(key, val); }

  /**
   * Logs double to 4 decimal points
   *
   * @param key Key to log, must be in headings vector passed in constructor
   * @param val Value to log
   */
  void LogNum(std::string key, double val) { m_csv.LogNum(key, val); }

  /**
   * Logs double to specified precision
   *
   * @param key Key to log, must be in headings vector passed in constructor
   * @param val Value to log
   * @param precision Precision
   */
  void LogNum(std::string key, double val, int precision) {
    m_csv.LogNum(key, val, precision);
  }

  /**
   * Logs boolean
   *
   * @param key Key to log, must be in headings vector passed in constructor
   * @param val Value to log
   */
  void LogBool(std::string key, bool val) { m_csv.LogBool(key, val); }

  /**
   * Logs a row of values
   *
   * @note size of row must be same as heading vector size
   *
   * @param row The row of values
   */
  void LogRow(std::vector<std::string> row) { m_csv.LogRow(row); }

  /**
   * Sets level
   *
   * @param level Level
   */
  void SetLevel(LogLevel level) { m_file.SetLevel(level); }

  /**
   * Sets if should log to console
   *
   * @param logToConsole true if should, false if not
   */
  void SetLogToConsole(bool logToConsole) {
    m_file.SetLogToConsole(logToConsole);
  }

  /**
   * Gets if should log to console
   *
   * @returns if should log to console
   */
  bool GetLogToConsole() const { return m_file.GetLogToConsole(); }

  /**
   * Gets current level
   *
   * @returns current level
   */
  LogLevel GetLevel() const { return m_file.GetLevel(); }

  /**
   * Logs trace
   *
   * @param heading Heading/title, to categorize log
   * @param msg Log message
   */
  void Trace(std::string heading, std::string msg) {
    m_file.Trace(heading, msg);
  }

  /**
   * Logs debug
   *
   * @param heading Heading/title, to categorize log
   * @param msg Log message
   */
  void Debug(std::string heading, std::string msg) {
    m_file.Debug(heading, msg);
  }

  /**
   * Logs info
   *
   * @param heading Heading/title, to categorize log
   * @param msg Log message
   */
  void Info(std::string heading, std::string msg) { m_file.Info(heading, msg); }

  /**
   * Logs warn
   *
   * @param heading Heading/title, to categorize log
   * @param msg Log message
   */
  void Warn(std::string heading, std::string msg) { m_file.Warn(heading, msg); }

  /**
   * Logs error
   *
   * @param heading Heading/title, to categorize log
   * @param msg Log message
   */
  void Error(std::string heading, std::string msg) {
    m_file.Error(heading, msg);
  }

  /**
   * Logs fatal
   *
   * @param heading Heading/title, to categorize log
   * @param msg Log message
   */
  void Fatal(std::string heading, std::string msg) {
    m_file.Fatal(heading, msg);
  }

private:
  struct stat m_st = {0};
  std::string FOLDER = "frclogs/";
  std::string m_dateStr = TimeMgr::GetTimeStr();
  CSVLogger m_csv;
  FileLogger m_file;
  bool m_initialized = false;
};