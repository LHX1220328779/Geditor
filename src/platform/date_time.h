
#pragma once

namespace geditor {

class DateTime {
 public:
  DateTime();

  ~DateTime();

  void SetToNow();

  void Set(int year, int month, int date);

  void Format(char *szBuffer, int bufSize);

 private:
  int sec_;
  int min_;
  int hour_;
  int day_;
  int mon_;
  int year_;
};

}  // namespace geditor
