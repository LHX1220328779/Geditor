#include "pcd/async_task.h"

namespace geditor {

bool AsyncTask::Execute() {
  func_ = std::make_shared<std::thread>(std::bind(&AsyncTask::execute, this));
  func_->join();
  return true;
}

void AsyncTask::execute() {
  onPreExecute();
  int ret = doInBackground();
  onPostExecute(ret != 0);
}

}  // namespace geditor
