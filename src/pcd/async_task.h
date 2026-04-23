
#pragma once

#include <functional>
#include <memory>
#include <thread>

namespace geditor {

// 异步任务
class AsyncTask {
 public:
  virtual bool Execute();

 protected:
  virtual void onProgressUpdate(int progress) = 0;

  virtual void onPreExecute() = 0;

  virtual int doInBackground() = 0;

  virtual void onPostExecute(bool result) = 0;

 private:
  void execute();

  std::shared_ptr<std::thread> func_;
};

}  // namespace geditor
