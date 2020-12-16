#ifndef TIMESTAMPS_LOGGER_H
#define TIMESTAMPS_LOGGER_H

#include <fstream>
#include <string>
#include <iostream>
#include <chrono>

#ifdef LOG_TRACKING_TIMESTAMPS
class TimestampsLogger {
  public:
    TimestampsLogger(std::string output_file_name) :
        file_name(output_file_name), idx(0) {
      f.open(output_file_name);
      f << std::fixed;
    }

    TimestampsLogger(void) : TimestampsLogger("TimestampsLogger.csv") {}

    ~TimestampsLogger() {
      f.close();
    }

    void Log(void) {
      f << idx << " " << std::chrono::duration_cast<std::chrono::microseconds>(
        std::chrono::high_resolution_clock::now().time_since_epoch()).count() *
        1.0e-6 << std::endl;
      std::cout << "|TimestampsLogger " << file_name << " " << idx++ << "|" <<
        std::endl;
    }

  private:
    size_t idx;
    std::string file_name;
    std::ofstream f;
};
#else
class TimestampsLogger {
  public:
    TimestampsLogger(std::string output_file_name) {}
    TimestampsLogger(void) {}
    ~TimestampsLogger() {}
    void Log(void) {}
};
#endif
#endif
