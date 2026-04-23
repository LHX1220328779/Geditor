#include "pathplanner/blanket/contourstracing.h"

#if CV_VERSION_MAJOR > 3
#define CV_CHAIN_APPROX_SIMPLE cv::CHAIN_APPROX_SIMPLE
#define CV_RETR_CCOMP cv::RETR_CCOMP
#endif

using namespace std;
using namespace cv;
using namespace ccpp;

/**********************************************
 * Function: ContoursTracing
 * Description: the construct function for ContoursTracing
 * Input: None
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
ContoursTracing::ContoursTracing() {}

/**********************************************
 * Function: ContoursTracing
 * Description: the construct function for ContoursTracing
 * Input: file path storing the basemap
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
ContoursTracing::ContoursTracing(const std::string& filepath) {
  // init basic param
  moving_step_ = 1;
  free_index_ = 0;
  obs_index_ = 0;

  filepath_ = filepath;
  orig_image_ = imread(filepath_).clone();
  color_image_ = orig_image_.clone();
  rows_ = orig_image_.rows;
  cols_ = orig_image_.cols;
  binary_flag_ = Color2Binary();
}

/**********************************************
 * Function: ContoursTracing
 * Description: the construct function for ContoursTracing
 * Input: file path storing the basemap
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
ContoursTracing::ContoursTracing(const std::string& filepath,
                                 const int& index) {
  // init basic param
  moving_step_ = 1;
  free_index_ = 0;
  obs_index_ = 0;

  filepath_ = filepath;
  orig_image_ = imread(filepath_).clone();
  color_image_ = orig_image_.clone();
  rows_ = orig_image_.rows;
  cols_ = orig_image_.cols;
  binary_flag_ = Color2Binary(index);
}

ContoursTracing::ContoursTracing(const cv::Mat& origin) {
  moving_step_ = 1;
  free_index_ = 0;
  obs_index_ = 0;

  rows_ = origin.rows;
  cols_ = origin.cols;
  binary_flag_ = true;
  binary_image_ = origin.clone();
  binary_image_clone_ = binary_image_.clone();
  binary_image_clone_test_ = binary_image_.clone();
  single_contour_mat_ = binary_image_.clone();
  ClearContourMat();
}

/**********************************************
 * Function: ContoursTracing
 * Description: the construct function for ContoursTracing
 * Input: file path storing the basemap
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
ContoursTracing::ContoursTracing(const std::string& filepath, const int& x,
                                 const int& y) {
  // init basic param
  moving_step_ = 1;
  free_index_ = 0;
  obs_index_ = 0;

  filepath_ = filepath;
  orig_image_ = imread(filepath_).clone();
  color_image_ = orig_image_.clone();
  rows_ = orig_image_.rows;
  cols_ = orig_image_.cols;
  curx_ = x;
  cury_ = y;
  binary_flag_ = FloodFilling();
}

/**********************************************
 * Function: ContoursTracing
 * Description: the destruct function for ContoursTracing
 * Input: None
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
ContoursTracing::~ContoursTracing() {
  // TODO(zhangfuqiang)
}

/**********************************************
 * Function: ContoursDetection
 * Description: 1. detect the contours from the map(default binary_map_clone_)
                2. detect the connors from the map(default binary_map_clone_)
 * Input: None
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
bool ContoursTracing::ContoursDetection() {
  if (!binary_flag_) {
    std::cout << "binarize failed" << std::endl;
    return false;
  }

  std::vector<std::vector<Point> > contours;
  std::vector<Vec4i> hierarchy;
  findContours(binary_image_clone_, contours, hierarchy, CV_RETR_CCOMP,
               CV_CHAIN_APPROX_SIMPLE);
  std::cout << "number od orig image is:" << contours.size() << std::endl;
  contours_.clear();
  for (int i = 0; i < contours.size(); i++) {
    std::cout << contours[i].size() << std::endl;
    if (contours[i].size() > 10) {
      contours_.push_back(contours[i]);
    }
  }
  drawContours(binary_image_clone_, contours_, -1, cv::Scalar::all(255));

  // param for cornor detection
  std::vector<cv::Point> corners;
  int max_corners = 150;
  double quality_level = 0.01;
  double min_distance = 20.0;
  int block_size = 3;
  bool use_harris = false;
  double k = 0.04;

  // cornor detection
  goodFeaturesToTrack(binary_image_clone_, corners, max_corners, quality_level,
                      min_distance, cv::Mat(), block_size, use_harris, k);
  for (int i = 0; i < corners.size(); i++) {
    circle(binary_image_clone_, corners[i], 1, cv::Scalar(255), 2, 8, 0);
  }
  return true;
}

/**********************************************
 * Function: SliceDecomposition
 * Description: slice decomposition, sweeping a line from the left of a map to
 the right, and then intersects a number of freespce regions and obstacle
 regions
 * Input: None
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
bool ContoursTracing::SliceDecomposition() {
  if (!binary_flag_) {
    std::cout << "binarize failed" << std::endl;
    return false;
  }
  static int test_i = 0;
  std::cout << "row is:" << rows_ << ",col is:" << cols_ << std::endl;
  int a = binary_image_.at<uchar>(1, 1);
  std::cout << a << std::endl;
  // abstract the largest contours from the image
  bool boundary_flag = RectInterception();
  std::cout << boundary_[0] << "," << boundary_[1] << "," << boundary_[2] << ","
            << boundary_[3] << std::endl;
  int col_min = boundary_[0];
  int col_max = boundary_[2];
  Slice pre_slice = GetSingleSlice(col_min);
  bool test_flag = pre_slice.intersection.size() == pre_slice.segment_num;

  for (int i = 0; i < pre_slice.segment_num; i++) {
    int pre_slice_pixel = pre_slice.intersection[i].cur_pixel;
    cout << "check point2" << endl;
    if (pre_slice_pixel == 255) {
      // free space region
      for (int j = pre_slice.intersection[i].start_num;
           j <= pre_slice.intersection[i].end_num; j++) {
        free_contours_[free_index_].push_back(cv::Point(col_min, j));
      }
      pre_slice.intersection[i].region_num = free_index_;
      free_index_++;
    } else {
      // cout << "first" << pre_slice.intersection[i].start_num << "," <<
      // pre_slice.intersection[i].end_num << endl;
      for (int j = pre_slice.intersection[i].start_num;
           j <= pre_slice.intersection[i].end_num; j++) {
        obs_contours_[obs_index_].push_back(cv::Point(col_min, j));
      }
      pre_slice.intersection[i].region_num = obs_index_;
      obs_index_++;
    }
  }

  for (int i = col_min + moving_step_; i <= col_max; i += moving_step_) {
    Slice cur_slice = GetSingleSlice(i);
    // cout << "first" << i << "," << cur_slice.intersection[0].start_num << ","
    //                             << cur_slice.intersection[0].end_num << ","
    //                             << cur_slice.intersection[0].region_num <<","
    //                             << cur_slice.intersection.size() << endl;
    // check the difference between the last and the current slice
    bool region_flag = SetRegionNum(pre_slice, cur_slice, i);
    pre_slice = cur_slice;
    // cout << "third" << i << "," << pre_slice.intersection[0].start_num << ","
    //                             << pre_slice.intersection[0].end_num << ","
    //                             << pre_slice.intersection[0].region_num <<","
    //                             << pre_slice.intersection.size() << endl;
  }
  ContoursOptimization();
  cout << "free_index:" << free_index_ << "," << free_contours_.size() << endl;
  cout << "obs_index:" << obs_index_ << "," << obs_contours_.size() << endl;
  cout << "test_i:" << test_i << endl;
  return true;
}

/**********************************************
 * Function: SliceDecomposition
 * Description: slice decomposition, sweeping a line from the left of a map to
 the right, and then intersects a number of freespce regions and obstacle
 regions
 * Input: None
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
bool ContoursTracing::SliceDecomposition(const bool& fullflag) {
  if (!binary_flag_) {
    std::cout << "binarize failed" << std::endl;
    return false;
  }
  static int test_i = 0;
  std::cout << "row is:" << rows_ << ",col is:" << cols_ << std::endl;
  int a = binary_image_.at<uchar>(1, 1);
  std::cout << a << std::endl;
  // abstract the largest contours from the image
  bool boundary_flag = RectInterception();
  std::cout << boundary_[0] << "," << boundary_[1] << "," << boundary_[2] << ","
            << boundary_[3] << std::endl;
  int col_min = boundary_[0];
  int col_max = boundary_[2];
  Slice pre_slice = GetSingleSlice(col_min);
  bool test_flag = pre_slice.intersection.size() == pre_slice.segment_num;

  for (int i = 0; i < pre_slice.segment_num; i++) {
    int pre_slice_pixel = pre_slice.intersection[i].cur_pixel;
    cout << "check point2" << endl;
    if (pre_slice_pixel == 255) {
      // free space region
      for (int j = pre_slice.intersection[i].start_num;
           j <= pre_slice.intersection[i].end_num; j++) {
        free_contours_[free_index_].push_back(cv::Point(col_min, j));
      }
      pre_slice.intersection[i].region_num = free_index_;
      free_index_++;
    } else {
      // cout << "first" << pre_slice.intersection[i].start_num << "," <<
      // pre_slice.intersection[i].end_num << endl;
      for (int j = pre_slice.intersection[i].start_num;
           j <= pre_slice.intersection[i].end_num; j++) {
        obs_contours_[obs_index_].push_back(cv::Point(col_min, j));
      }
      pre_slice.intersection[i].region_num = obs_index_;
      obs_index_++;
    }
  }

  for (int i = col_min + moving_step_; i <= col_max; i += moving_step_) {
    Slice cur_slice = GetSingleSlice(i);
    // cout << "first" << i << "," << cur_slice.intersection[0].start_num << ","
    //                             << cur_slice.intersection[0].end_num << ","
    //                             << cur_slice.intersection[0].region_num <<","
    //                             << cur_slice.intersection.size() << endl;
    // check the difference between the last and the current slice
    bool region_flag = SetRegionNum(pre_slice, cur_slice, i, fullflag);
    pre_slice = cur_slice;
    // cout << "third" << i << "," << pre_slice.intersection[0].start_num << ","
    //                             << pre_slice.intersection[0].end_num << ","
    //                             << pre_slice.intersection[0].region_num <<","
    //                             << pre_slice.intersection.size() << endl;
  }
  cout << "free_index:" << free_index_ << "," << free_contours_.size() << endl;
  cout << "obs_index:" << obs_index_ << "," << obs_contours_.size() << endl;
  cout << "test_i:" << test_i << endl;
  return true;
}

/**********************************************
 * Function: SingleContoursDetection
 * Description: when we have devide the whole region into some seperate zones,
 *              we need to get the contours of each zones.
 * Input: index----represent the freespace region to detect
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
bool ContoursTracing::SingleContoursDetection() {
  std::vector<std::vector<Point> > contours;
  std::vector<std::vector<Point> > contours_fix;
  std::vector<Vec4i> hierarchy;
  findContours(single_contour_mat_, contours, hierarchy, CV_RETR_CCOMP,
               CV_CHAIN_APPROX_SIMPLE);
  std::cout << "number od orig image is:" << contours.size() << std::endl;
  contours_fix.clear();
  for (int i = 0; i < contours.size(); i++) {
    std::cout << contours[i].size() << std::endl;
  }
  // contours_fix.push_back(contours[0]);
  drawContours(single_contour_mat_, contours, -1, cv::Scalar::all(255));
  return true;
}

/**********************************************
 * Function: SinglePathPlanning
 * Description:
 * Input: index----represent the freespace region to detect
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
std::vector<cv::Point> ContoursTracing::SinglePathPlanning(const int& index) {
  return std::vector<cv::Point>();
}

/**********************************************
 * Function: Color2Binary
 * Description: convert the basemap to a gray map and a binary map
 * Input: None
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
bool ContoursTracing::Color2Binary() {
  if (orig_image_.empty()) {
    std::cout << "orig image is empty" << std::endl;
    return false;
  }

  // turn to the gray image
  cvtColor(orig_image_, gray_image_, COLOR_BGR2GRAY);
  threshold(gray_image_, binary_image_, 145, 255, THRESH_BINARY);
  binary_image_clone_ = binary_image_.clone();
  binary_image_clone_test_ = binary_image_.clone();
  single_contour_mat_ = binary_image_.clone();
  ClearContourMat();
  return true;
}

/**********************************************
 * Function: Color2Binary
 * Description: convert the basemap to a gray map and a binary map
 * Input: index----the number of the cleaning region
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
bool ContoursTracing::Color2Binary(const int& index) {
  // TODO(zhangfuqiang)
  if (orig_image_.empty()) {
    std::cout << "orig image is empty" << std::endl;
    return false;
  }
  for (int i = 0; i < rows_; i++) {
    for (int j = 0; j < cols_; j++) {
      if (orig_image_.at<Vec3b>(i, j)[0] == index &&
          orig_image_.at<Vec3b>(i, j)[1] == 100 &&
          orig_image_.at<Vec3b>(i, j)[2] == 110) {
        orig_image_.at<Vec3b>(i, j)[0] = 0;
        orig_image_.at<Vec3b>(i, j)[1] = 255;
        orig_image_.at<Vec3b>(i, j)[2] = 0;
      } else {
        orig_image_.at<Vec3b>(i, j)[0] = 0;
        orig_image_.at<Vec3b>(i, j)[1] = 0;
        orig_image_.at<Vec3b>(i, j)[2] = 0;
      }
    }
  }
  // turn to the gray image
  cvtColor(orig_image_, gray_image_, COLOR_BGR2GRAY);
  threshold(gray_image_, binary_image_, 145, 255, THRESH_BINARY);
  binary_image_clone_ = binary_image_.clone();
  binary_image_clone_test_ = binary_image_.clone();
  single_contour_mat_ = binary_image_.clone();
  ClearContourMat();
  return true;
}

/**********************************************
 * Function: FloodFilling
 * Description: detectsingleregion
 * Input: None
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
bool ContoursTracing::FloodFilling() { return false; }

/**********************************************
 * Function: ShowImage
 * Description: show all pictures
 * Input: None
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
bool ContoursTracing::ShowImage() {
  // ShowOrigImage();
  // ShowGrayImage();
  ShowBinaryImage();
  // ShowBinaryTestImage();
  // ShowSingleContour("obs");
  waitKey(0);
  return true;
}

/**********************************************
 * Function: ShowOrigImage
 * Description: show the original basemap
 * Input: None
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
bool ContoursTracing::ShowOrigImage() {
  if (orig_image_.empty()) {
    std::cout << "orig image is empty" << std::endl;
    return false;
  }
  namedWindow("original image", WINDOW_NORMAL);
  imshow("original image", orig_image_);
  return true;
}

/**********************************************
 * Function: ShowGrayImage
 * Description: show the gray map
 * Input: None
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
bool ContoursTracing::ShowGrayImage() {
  if (gray_image_.empty()) return false;
  imwrite("gray.jpg", gray_image_);
  namedWindow("gray image", WINDOW_NORMAL);
  imshow("gray image", gray_image_);
  return true;
}

/**********************************************
 * Function: ShowBinaryImage
 * Description: show the binary map
 * Input: None
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
bool ContoursTracing::ShowBinaryImage() {
  if (binary_image_.empty()) return false;
  imwrite("binary.jpg", binary_image_);
  namedWindow("binary image", WINDOW_NORMAL);
  imshow("binary image", binary_image_);
  return true;
}

/**********************************************
 * Function: ShowBinaryTestImage
 * Description: for test,show the binary map
 * Input: None
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
bool ContoursTracing::ShowBinaryTestImage() {
  if (binary_image_clone_test_.empty()) return false;
  imwrite("binarytest.jpg", binary_image_);
  namedWindow("binary image test", WINDOW_NORMAL);
  imshow("binary image test", binary_image_clone_test_);
  return true;
}

/**********************************************
 * Function: ShowBinaryTestImage
 * Description: for test,show the binary map
 * Input: None
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
bool ContoursTracing::ShowSingleContour(std::string name) {
  if (single_contour_mat_.empty()) return false;
  // imwrite("singlecontour.jpg", binary_image_);
  namedWindow(name, WINDOW_NORMAL);
  imshow(name, single_contour_mat_);
  return true;
}

/**********************************************
 * Function: ClearContourMat
 * Description: for test, clear the pixel of single_contour_mat_.
 * Input: None
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
bool ContoursTracing::ClearContourMat() {
  for (int i = 0; i < rows_; i++) {
    for (int j = 0; j < cols_; j++) {
      single_contour_mat_.at<uchar>(i, j) = 0;
    }
  }
  return true;
}

/**********************************************
 * Function: InsideFreeSpace
 * Description: check if the point is in the freespace
 * Input: row----the row number of the point;
          col----the col number of the point;
 * Output: None
 * Return: true----the point is in the freespace;
           false----the point is not in the freespace;
 * Others: None
 **********************************************/
bool ContoursTracing::InsideFreeSpace(const int& row, const int& col) {
  if (row > rows_ || row < 0) {
    std::cout << "the row number is invalid" << std::endl;
    return false;
  }
  if (col > cols_ || col < 0) {
    std::cout << "the col number is invalid" << std::endl;
    return false;
  }
  int binary_px = binary_image_.at<uchar>(row, col);
  // 0:black, represent the obstacle region;
  // 255:white, represent the free space;
  if (binary_px == 0) {
    return false;
  } else if (binary_px == 255) {
    return true;
  } else {
    std::cout << "the image is not binarilized" << std::endl;
    return false;
  }
}

/**********************************************
 * Function: GetSingleSlice
 * Description: get the slices from a line
 * Input: col----the col number of the point
 * Output: None
 * Return: the slice of the line
 * Others: None
 **********************************************/
Slice ContoursTracing::GetSingleSlice(const int& col) {
  Slice single_slice;
  int init_px = binary_image_.at<uchar>(boundary_[1], col);
  single_slice.init_pixel = init_px;
  // single_slice.intersection.push_back(boundary_[1]);
  int single_px = init_px;
  int segment_num = 1;
  int lastindex = boundary_[1];
  for (int i = boundary_[1]; i <= boundary_[3]; i++) {
    int temp_px = binary_image_.at<uchar>(i, col);
    if (temp_px != single_px) {
      segment_num++;
      single_slice.intersection.push_back(
          Segment(lastindex, i - 1, single_px, 0));
      single_px = temp_px;
      lastindex = i;
    }
  }
  single_slice.intersection.push_back(
      Segment(lastindex, boundary_[3], single_px, 0));
  single_slice.segment_num = segment_num;
  if (single_slice.segment_num == 0) {
    cout << "zfq::slice error" << endl;
  } else if (single_slice.segment_num == 1) {
    if (single_slice.init_pixel != 0) {
      cout << "zfq::slice error 1" << endl;
    }
    return single_slice;
  }
  for (int i = 0; i < single_slice.segment_num - 1; i++) {
    if (single_slice.intersection[i].cur_pixel ==
        single_slice.intersection[i + 1].cur_pixel) {
      cout << "zfq::slice error 2" << endl;
    }
  }
  return single_slice;
}

/**********************************************
 * Function: RectInterception
 * Description: detect the left, right, top, bottom of the largest contour
 * Input: None
 * Output: None
 * Return: true----there is a explicit contour
           false----there is no largest contours
 * Others: None
 **********************************************/
bool ContoursTracing::RectInterception() {
  std::vector<int> boundary_col;
  std::vector<int> boundary_row;
  boundary_.clear();
  if (contours_.empty()) {
    std::cout << "the image has no contours" << std::endl;
    boundary_.push_back(0);
    boundary_.push_back(0);
    boundary_.push_back(rows_ - 1);
    boundary_.push_back(cols_ - 1);
    return false;
  }
  std::cout << "size is" << contours_[0].size() << std::endl;
  for (int i = 0; i < contours_[0].size(); i++) {
    boundary_col.push_back(contours_[0][i].x);
    boundary_row.push_back(contours_[0][i].y);
  }
  std::vector<int>::iterator smallest_x =
      std::min_element(std::begin(boundary_col), std::end(boundary_col));
  std::vector<int>::iterator smallest_y =
      std::min_element(std::begin(boundary_row), std::end(boundary_row));
  std::vector<int>::iterator biggest_x =
      std::max_element(std::begin(boundary_col), std::end(boundary_col));
  std::vector<int>::iterator biggest_y =
      std::max_element(std::begin(boundary_row), std::end(boundary_row));
  int min_x = (*smallest_x) - 5 <= 0 ? 0 : (*smallest_x) - 5;
  int min_y = (*smallest_y) - 5 <= 0 ? 0 : (*smallest_y) - 5;
  int max_x = (*biggest_x) + 5 >= cols_ - 1 ? cols_ - 1 : (*biggest_x) + 5;
  int max_y = (*biggest_y) + 5 >= rows_ - 1 ? rows_ - 1 : (*biggest_y) + 5;
  boundary_.push_back(min_x);
  boundary_.push_back(min_y);
  boundary_.push_back(max_x);
  boundary_.push_back(max_y);
  return true;
}

/**********************************************
 * Function: IntersectionDetection
 * Description: check if there exist a intersection set between two segments
 * Input: first_source----the head index of the first segment
          first_dest----the tail index of the first segment
          second_source----the head index of the second segment
          second_dest----the tail index of the second segment
 * Output: None
 * Return: 0----false,represent there is no intersection between two segment
           1----true,represent there is intersection between two segment
           2----represent the input error,no use
 * Others: None
 **********************************************/
int ContoursTracing::IntersectionDetection(const int& first_source,
                                           const int& first_dest,
                                           const int& second_source,
                                           const int& second_dest) {
  if ((first_dest < first_source) || (second_dest < second_source))
    return 2;  // TODO(zhangfuqiang) represent the input error,no use
  if ((first_source > second_dest) || (first_dest < second_source)) {
    return 0;  // false,represent there is no intersection between two segment
  }
  return 1;  // true,represent there is intersection between two segment
}

/**********************************************
 * Function: GetSegemnts
 * Description: search the start segment and the end segment of the slice
                checking the segment and the slice
 * Input: c_segment----the segment need to check
          c_slice----the slice need to tracerse
 * Output: None
 * Return: the start segment and the end segment of the slice
 * Others: None
 **********************************************/
std::vector<int> ContoursTracing::GetSegemnts(const ccpp::Segment& c_segment,
                                              const ccpp::Slice& c_slice) {
  int start = c_segment.start_num;
  int end = c_segment.end_num;
  int start_index, end_index;
  std::vector<int> result;
  for (int i = 0; i < c_slice.intersection.size(); i++) {
    if (start >= c_slice.intersection[i].start_num &&
        start <= c_slice.intersection[i].end_num) {
      start_index = i;
      break;
    }
  }
  for (int i = start_index; i < c_slice.intersection.size(); i++) {
    if (end >= c_slice.intersection[i].start_num &&
        end <= c_slice.intersection[i].end_num) {
      end_index = i;
      break;
    }
  }
  if (start_index != end_index) {
    if (c_slice.intersection[start_index].cur_pixel != c_segment.cur_pixel)
      start_index++;
    if (c_slice.intersection[end_index].cur_pixel != c_segment.cur_pixel)
      end_index--;
  }
  result.push_back(start_index);
  result.push_back(end_index);
  // cout << "start:" << start_index << ",end:" << end_index << endl;
  return result;
}

/**********************************************
 * Function: SetRegionNum
 * Description:
 * Input: pre_slice----the last slice of the sweeping line
 *        cur_slice----the current slice of the sweeping line
 * Output: None
 * Return: true, no use
 * Others: None
 **********************************************/
bool ContoursTracing::SetRegionNum(ccpp::Slice& pre_slice,
                                   ccpp::Slice& cur_slice, const int& cur_col) {
  // std::cout << "set region" << cur_col << std::endl;
  int inter_flag = true;
  int begin_index;
  int next_index;
  int begin_index_cur;
  int next_index_cur;
  int first_break;
  int consecutive_count = 0;
  bool intersection_flag = false;
  bool cur_extend_flag = false;
  std::vector<int> detected_break;
  std::vector<int> detected_break_cur;
  // cout << "pre-num:"
  // <<pre_slice.segment_num<<",col"<<cur_col-moving_step_<<endl; for (int i =
  // 0; i < pre_slice.segment_num; i++) {
  //   cout<<i<<":"<<pre_slice.intersection[i].start_num<<","
  //               <<pre_slice.intersection[i].end_num<<","
  //               <<pre_slice.intersection[i].region_num<<","
  //               <<pre_slice.intersection[i].cur_pixel<<endl;
  // }
  // cout << "cur-num:" <<cur_slice.segment_num<<",col:"<<cur_col<<endl;
  // for (int i = 0; i < cur_slice.segment_num; i++) {
  //   cout<<i<<":"<<cur_slice.intersection[i].start_num<<","
  //               <<cur_slice.intersection[i].end_num<<","
  //               <<cur_slice.intersection[i].region_num<<","
  //               <<cur_slice.intersection[i].cur_pixel<<endl;
  // }
  if (pre_slice.segment_num == cur_slice.segment_num) {
    for (int i = 0; i < pre_slice.intersection.size(); i++) {
      std::vector<int> init_con_num =
          GetSegemnts(pre_slice.intersection[i], cur_slice);
      if (init_con_num[1] - init_con_num[0] != 0) {
        inter_flag = false;
        break;
      }
    }
  }
  // cout << cur_col << "," << inter_flag << endl;
  if (inter_flag && pre_slice.segment_num == cur_slice.segment_num) {
    // cout << "the neighbor is similar" << endl;
    // inter_flag=1 means that every corresponding segment between two slices is
    // intersected, so we believe that the sweeping line has no change.
    if (cur_col == boundary_[2]) {
      for (int i = 0; i < pre_slice.intersection.size(); i++) {
        int pre_region_num = pre_slice.intersection[i].region_num;
        cur_slice.intersection[i].region_num = pre_region_num;
        if (cur_slice.intersection[i].cur_pixel == 255) {
          for (int j = cur_slice.intersection[i].start_num;
               j <= cur_slice.intersection[i].end_num; j++) {
            free_contours_[pre_region_num].push_back(cv::Point(cur_col, j));
          }
        } else {
          for (int j = cur_slice.intersection[i].start_num;
               j <= cur_slice.intersection[i].end_num; j++) {
            obs_contours_[pre_region_num].push_back(cv::Point(cur_col, j));
          }
        }
      }
      return true;
    }
    for (int i = 0; i < pre_slice.intersection.size(); i++) {
      int pre_region_num = pre_slice.intersection[i].region_num;
      cur_slice.intersection[i].region_num = pre_region_num;
      // cout << pre_region_num << endl;
      if (cur_slice.intersection[i].cur_pixel == 255) {
        if (cur_slice.intersection[i].start_num ==
            cur_slice.intersection[i].end_num) {
          free_contours_[pre_region_num].push_back(
              cv::Point(cur_col, cur_slice.intersection[i].start_num));
        } else {
          free_contours_[pre_region_num].push_back(
              cv::Point(cur_col, cur_slice.intersection[i].start_num));
          free_contours_[pre_region_num].push_back(
              cv::Point(cur_col, cur_slice.intersection[i].end_num));
        }
      } else {
        if (cur_slice.intersection[i].start_num ==
            cur_slice.intersection[i].end_num) {
          obs_contours_[pre_region_num].push_back(
              cv::Point(cur_col, cur_slice.intersection[i].start_num));
        } else {
          obs_contours_[pre_region_num].push_back(
              cv::Point(cur_col, cur_slice.intersection[i].start_num));
          obs_contours_[pre_region_num].push_back(
              cv::Point(cur_col, cur_slice.intersection[i].end_num));
        }
      }
    }
  } else {
    int i = 0;
    int j = 0;
    int pre2curdiff, cur2prediff;
    std::vector<int> temp_con_num;
    while (i < pre_slice.intersection.size() ||
           j < cur_slice.intersection.size()) {
      std::vector<int> pre_con_num =
          GetSegemnts(pre_slice.intersection[i], cur_slice);
      std::vector<int> cur_con_num =
          GetSegemnts(cur_slice.intersection[j], pre_slice);
      pre2curdiff = pre_con_num[1] - pre_con_num[0];
      cur2prediff = cur_con_num[1] - cur_con_num[0];
      if (pre2curdiff == 0 && cur2prediff == 0) {
        // TODO(zhangfuqiang)
        cur_slice.intersection[j].region_num =
            pre_slice.intersection[i].region_num;
        if (cur_slice.intersection[j].cur_pixel == 255) {
          if (cur_slice.intersection[j].start_num ==
              cur_slice.intersection[j].end_num) {
            free_contours_[cur_slice.intersection[j].region_num].push_back(
                cv::Point(cur_col, cur_slice.intersection[j].start_num));
          } else {
            free_contours_[cur_slice.intersection[j].region_num].push_back(
                cv::Point(cur_col, cur_slice.intersection[j].start_num));
            free_contours_[cur_slice.intersection[j].region_num].push_back(
                cv::Point(cur_col, cur_slice.intersection[j].end_num));
          }
        } else {
          if (cur_slice.intersection[j].start_num ==
              cur_slice.intersection[j].end_num) {
            obs_contours_[cur_slice.intersection[j].region_num].push_back(
                cv::Point(cur_col, cur_slice.intersection[j].start_num));
          } else {
            obs_contours_[cur_slice.intersection[j].region_num].push_back(
                cv::Point(cur_col, cur_slice.intersection[j].start_num));
            obs_contours_[cur_slice.intersection[j].region_num].push_back(
                cv::Point(cur_col, cur_slice.intersection[j].end_num));
          }
        }
        i++;
        j++;
      } else {
        // TODO(zhangfuqiang) has done
        if (pre2curdiff > 0) {
          if (pre_con_num[1] > j) {
            begin_index = i;
            begin_index_cur = j;
            intersection_flag = true;
            cur_extend_flag = true;
            j = pre_con_num[1];

            while (intersection_flag) {
              temp_con_num.clear();
              if (cur_extend_flag) {
                temp_con_num =
                    GetSegemnts(cur_slice.intersection[j], pre_slice);
              } else {
                temp_con_num =
                    GetSegemnts(pre_slice.intersection[i], cur_slice);
              }
              if (temp_con_num[1] - temp_con_num[0] == 0) {
                // enter a temp stable status, draw line
                detected_break.push_back(begin_index);
                detected_break.push_back(i);
                detected_break_cur.push_back(begin_index_cur);
                detected_break_cur.push_back(j);
                i++;
                j++;
                intersection_flag = false;
                continue;
              } else {
                if (cur_extend_flag) {
                  i = temp_con_num[1];
                  cur_extend_flag = false;
                } else {
                  j = temp_con_num[1];
                  cur_extend_flag = true;
                }
              }
            }
          }
        } else if (cur2prediff > 0) {
          if (cur_con_num[1] > i) {
            begin_index = i;
            begin_index_cur = j;
            intersection_flag = true;
            cur_extend_flag = false;
            i = cur_con_num[1];
            while (intersection_flag) {
              temp_con_num.clear();
              if (cur_extend_flag) {
                temp_con_num =
                    GetSegemnts(cur_slice.intersection[j], pre_slice);
              } else {
                temp_con_num =
                    GetSegemnts(pre_slice.intersection[i], cur_slice);
              }
              if (temp_con_num[1] - temp_con_num[0] == 0) {
                // enter a temp stable status, draw line
                detected_break.push_back(begin_index);
                detected_break.push_back(i);
                detected_break_cur.push_back(begin_index_cur);
                detected_break_cur.push_back(j);
                i++;
                j++;
                intersection_flag = false;
                continue;
              } else {
                if (cur_extend_flag) {
                  i = temp_con_num[1];
                  cur_extend_flag = false;
                } else {
                  j = temp_con_num[1];
                  cur_extend_flag = true;
                }
              }
            }
          }
        }
      }
    }
    // Default ,the detected_break and the detected_break_cur is not empty
    for (i = 0; i < detected_break.size(); i += 2) {
      for (j = detected_break[i]; j <= detected_break[i + 1]; j++) {
        if (pre_slice.intersection[j].end_num -
                pre_slice.intersection[j].start_num >
            1) {
          if (pre_slice.intersection[j].cur_pixel == 255) {
            for (int k = pre_slice.intersection[j].start_num + 1;
                 k < pre_slice.intersection[j].end_num; k++) {
              free_contours_[pre_slice.intersection[j].region_num].push_back(
                  cv::Point(cur_col - moving_step_, k));
            }
          } else {
            for (int k = pre_slice.intersection[j].start_num + 1;
                 k < pre_slice.intersection[j].end_num; k++) {
              obs_contours_[pre_slice.intersection[j].region_num].push_back(
                  cv::Point(cur_col - moving_step_, k));
            }
          }
        }
      }
    }
    for (i = 0; i < detected_break_cur.size(); i += 2) {
      for (j = detected_break_cur[i]; j <= detected_break_cur[i + 1]; j++) {
        if (cur_slice.intersection[j].cur_pixel == 255) {
          for (int k = cur_slice.intersection[j].start_num;
               k <= cur_slice.intersection[j].end_num; k++) {
            free_contours_[free_index_].push_back(cv::Point(cur_col, k));
          }
          cur_slice.intersection[j].region_num = free_index_;
          free_index_++;
        } else {
          for (int k = cur_slice.intersection[j].start_num;
               k <= cur_slice.intersection[j].end_num; k++) {
            obs_contours_[obs_index_].push_back(cv::Point(cur_col, k));
          }
          // cout << j << "obs_index_" << obs_index_ << ",cur col:" << cur_col
          // << endl;
          cur_slice.intersection[j].region_num = obs_index_;
          obs_index_++;
        }
      }
    }
  }
  return true;
}

/**********************************************
 * Function: SetRegionNum
 * Description:
 * Input: pre_slice----the last slice of the sweeping line
 *        cur_slice----the current slice of the sweeping line
 * Output: None
 * Return: true, no use
 * Others: None
 **********************************************/
bool ContoursTracing::SetRegionNum(ccpp::Slice& pre_slice,
                                   ccpp::Slice& cur_slice, const int& cur_col,
                                   const bool& fullflag) {
  // std::cout << "set region" << cur_col << std::endl;
  int inter_flag = true;
  int begin_index;
  int next_index;
  int begin_index_cur;
  int next_index_cur;
  int first_break;
  int consecutive_count = 0;
  bool intersection_flag = false;
  bool cur_extend_flag = false;
  std::vector<int> detected_break;
  std::vector<int> detected_break_cur;
  // cout << "pre-num:"
  // <<pre_slice.segment_num<<",col"<<cur_col-moving_step_<<endl; for (int i =
  // 0; i < pre_slice.segment_num; i++) {
  //   cout<<i<<":"<<pre_slice.intersection[i].start_num<<","
  //               <<pre_slice.intersection[i].end_num<<","
  //               <<pre_slice.intersection[i].region_num<<","
  //               <<pre_slice.intersection[i].cur_pixel<<endl;
  // }
  // cout << "cur-num:" <<cur_slice.segment_num<<",col:"<<cur_col<<endl;
  // for (int i = 0; i < cur_slice.segment_num; i++) {
  //   cout<<i<<":"<<cur_slice.intersection[i].start_num<<","
  //               <<cur_slice.intersection[i].end_num<<","
  //               <<cur_slice.intersection[i].region_num<<","
  //               <<cur_slice.intersection[i].cur_pixel<<endl;
  // }
  if (pre_slice.segment_num == cur_slice.segment_num) {
    for (int i = 0; i < pre_slice.intersection.size(); i++) {
      std::vector<int> init_con_num =
          GetSegemnts(pre_slice.intersection[i], cur_slice);
      if (init_con_num[1] - init_con_num[0] != 0) {
        inter_flag = false;
        break;
      }
    }
  }
  // cout << cur_col << "," << inter_flag << endl;
  if (inter_flag && pre_slice.segment_num == cur_slice.segment_num) {
    // cout << "the neighbor is similar" << endl;
    // inter_flag=1 means that every corresponding segment between two slices is
    // intersected, so we believe that the sweeping line has no change.
    if (cur_col == boundary_[2]) {
      for (int i = 0; i < pre_slice.intersection.size(); i++) {
        int pre_region_num = pre_slice.intersection[i].region_num;
        cur_slice.intersection[i].region_num = pre_region_num;
        if (cur_slice.intersection[i].cur_pixel == 255) {
          for (int j = cur_slice.intersection[i].start_num;
               j <= cur_slice.intersection[i].end_num; j++) {
            free_contours_[pre_region_num].push_back(cv::Point(cur_col, j));
          }
        } else {
          for (int j = cur_slice.intersection[i].start_num;
               j <= cur_slice.intersection[i].end_num; j++) {
            obs_contours_[pre_region_num].push_back(cv::Point(cur_col, j));
          }
        }
      }
      return true;
    }
    for (int i = 0; i < pre_slice.intersection.size(); i++) {
      int pre_region_num = pre_slice.intersection[i].region_num;
      cur_slice.intersection[i].region_num = pre_region_num;
      // cout << pre_region_num << endl;
      if (cur_slice.intersection[i].cur_pixel == 255) {
        for (int j = cur_slice.intersection[i].start_num;
             j <= cur_slice.intersection[i].end_num; j++) {
          free_contours_[pre_region_num].push_back(cv::Point(cur_col, j));
        }
      } else {
        for (int j = cur_slice.intersection[i].start_num;
             j <= cur_slice.intersection[i].end_num; j++) {
          obs_contours_[pre_region_num].push_back(cv::Point(cur_col, j));
        }
      }
    }
  } else {
    int i = 0;
    int j = 0;
    int pre2curdiff, cur2prediff;
    std::vector<int> temp_con_num;
    while (i < pre_slice.intersection.size() ||
           j < cur_slice.intersection.size()) {
      std::vector<int> pre_con_num =
          GetSegemnts(pre_slice.intersection[i], cur_slice);
      std::vector<int> cur_con_num =
          GetSegemnts(cur_slice.intersection[j], pre_slice);
      pre2curdiff = pre_con_num[1] - pre_con_num[0];
      cur2prediff = cur_con_num[1] - cur_con_num[0];
      if (pre2curdiff == 0 && cur2prediff == 0) {
        // TODO(zhangfuqiang)
        cur_slice.intersection[j].region_num =
            pre_slice.intersection[i].region_num;
        if (cur_slice.intersection[j].cur_pixel == 255) {
          for (int k = cur_slice.intersection[j].start_num;
               k < cur_slice.intersection[j].end_num; k++) {
            free_contours_[cur_slice.intersection[j].region_num].push_back(
                cv::Point(cur_col, k));
          }
        } else {
          for (int k = cur_slice.intersection[j].start_num;
               k < cur_slice.intersection[j].end_num; k++) {
            obs_contours_[cur_slice.intersection[j].region_num].push_back(
                cv::Point(cur_col, k));
          }
        }
        i++;
        j++;
      } else {
        // TODO(zhangfuqiang) has done
        if (pre2curdiff > 0) {
          if (pre_con_num[1] > j) {
            begin_index = i;
            begin_index_cur = j;
            intersection_flag = true;
            cur_extend_flag = true;
            j = pre_con_num[1];

            while (intersection_flag) {
              temp_con_num.clear();
              if (cur_extend_flag) {
                temp_con_num =
                    GetSegemnts(cur_slice.intersection[j], pre_slice);
              } else {
                temp_con_num =
                    GetSegemnts(pre_slice.intersection[i], cur_slice);
              }
              if (temp_con_num[1] - temp_con_num[0] == 0) {
                // enter a temp stable status, draw line
                detected_break.push_back(begin_index);
                detected_break.push_back(i);
                detected_break_cur.push_back(begin_index_cur);
                detected_break_cur.push_back(j);
                i++;
                j++;
                intersection_flag = false;
                continue;
              } else {
                if (cur_extend_flag) {
                  i = temp_con_num[1];
                  cur_extend_flag = false;
                } else {
                  j = temp_con_num[1];
                  cur_extend_flag = true;
                }
              }
            }
          }
        } else if (cur2prediff > 0) {
          if (cur_con_num[1] > i) {
            begin_index = i;
            begin_index_cur = j;
            intersection_flag = true;
            cur_extend_flag = false;
            i = cur_con_num[1];
            while (intersection_flag) {
              temp_con_num.clear();
              if (cur_extend_flag) {
                temp_con_num =
                    GetSegemnts(cur_slice.intersection[j], pre_slice);
              } else {
                temp_con_num =
                    GetSegemnts(pre_slice.intersection[i], cur_slice);
              }
              if (temp_con_num[1] - temp_con_num[0] == 0) {
                // enter a temp stable status, draw line
                detected_break.push_back(begin_index);
                detected_break.push_back(i);
                detected_break_cur.push_back(begin_index_cur);
                detected_break_cur.push_back(j);
                i++;
                j++;
                intersection_flag = false;
                continue;
              } else {
                if (cur_extend_flag) {
                  i = temp_con_num[1];
                  cur_extend_flag = false;
                } else {
                  j = temp_con_num[1];
                  cur_extend_flag = true;
                }
              }
            }
          }
        }
      }
    }
    for (i = 0; i < detected_break_cur.size(); i += 2) {
      for (j = detected_break_cur[i]; j <= detected_break_cur[i + 1]; j++) {
        if (cur_slice.intersection[j].cur_pixel == 255) {
          for (int k = cur_slice.intersection[j].start_num;
               k <= cur_slice.intersection[j].end_num; k++) {
            free_contours_[free_index_].push_back(cv::Point(cur_col, k));
          }
          cur_slice.intersection[j].region_num = free_index_;
          free_index_++;
        } else {
          for (int k = cur_slice.intersection[j].start_num;
               k <= cur_slice.intersection[j].end_num; k++) {
            obs_contours_[obs_index_].push_back(cv::Point(cur_col, k));
          }
          // cout << j << "obs_index_" << obs_index_ << ",cur col:" << cur_col
          // << endl;
          cur_slice.intersection[j].region_num = obs_index_;
          obs_index_++;
        }
      }
    }
  }
  return true;
}
/**********************************************
 * Function: ContoursOptimization
 * Description: There are some zones of discontinuity in the contours, so we
 *need to fix the missing points to make the contours better Input: None Output:
 *None Return: None Others: only for the freespace region presently
 **********************************************/
bool ContoursTracing::ContoursOptimization() {
  std::map<int, std::set<int> > index_x;
  std::map<int, std::vector<cv::Point> >::iterator it;
  std::map<int, std::set<int> >::iterator it1;
  std::map<int, std::set<int> >::iterator it2;
  int pre_min;
  int pre_max;
  int cur_min;
  int cur_max;
  cout << "start optimize" << endl;
  for (it = free_contours_.begin(); it != free_contours_.end(); it++) {
    index_x.clear();

    for (int i = 0; i < it->second.size(); i++) {
      index_x[it->second[i].x].insert(it->second[i].y);
    }
    if (it->first == 0) {
      cout << "0" << (*--index_x.end()).first << (*index_x.begin()).first
           << endl;
    }
    if ((*--index_x.end()).first - (*index_x.begin()).first > 1) {
      for (it1 = index_x.begin(), it2 = std::next(index_x.begin());
           it2 != index_x.end(); it1++, it2++) {
        std::set<int>::iterator pre_min1 = std::min_element(
            index_x[it1->first].begin(), index_x[it1->first].end());
        std::set<int>::iterator pre_max1 = std::max_element(
            index_x[it1->first].begin(), index_x[it1->first].end());
        std::set<int>::iterator cur_min1 = std::min_element(
            index_x[it2->first].begin(), index_x[it2->first].end());
        std::set<int>::iterator cur_max1 = std::max_element(
            index_x[it2->first].begin(), index_x[it2->first].end());
        pre_min = *pre_min1;
        pre_max = *pre_max1;
        cur_min = *cur_min1;
        cur_max = *cur_max1;
        cout << "bound" << pre_min << "," << pre_max << "," << cur_min << ","
             << cur_max << endl;
        if (pre_min < cur_min) {
          for (int j = pre_min; j < cur_min; j++) {
            free_contours_[it->first].push_back(cv::Point(it1->first, j));
          }
        } else if (pre_min > cur_min) {
          for (int j = cur_min; j < pre_min; j++) {
            free_contours_[it->first].push_back(cv::Point(it2->first, j));
          }
        }
        if (pre_max < cur_max) {
          for (int j = pre_max; j < cur_max; j++) {
            free_contours_[it->first].push_back(cv::Point(it1->first, j));
          }
        } else if (pre_max > cur_max) {
          for (int j = cur_max; j < pre_max; j++) {
            free_contours_[it->first].push_back(cv::Point(it2->first, j));
          }
        }
      }
    }
  }
  return true;
}

/**********************************************
 * Function: GetContours
 * Description: get the largest contours from every subregion
 * Input: None
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
bool ContoursTracing::GetContours() {
  if (free_contours_.empty()) {
    return false;
  }
  std::vector<std::vector<Point> > contours;
  std::vector<Vec4i> hierarchy;

  for (int i = 0; i < free_contours_.size(); i++) {
    TestContourImage(i, 1);
    contours.clear();
    hierarchy.clear();
    findContours(single_contour_mat_, contours, hierarchy, RETR_EXTERNAL,
                 CV_CHAIN_APPROX_SIMPLE);
    if (contours.size() > 1 || contours.size() < 1) {
    std:
      cout << "contours fault:" << contours.size() << endl;
    }
  }

  std::cout << "the outer edge num is:" << contours.size() << std::endl;

  return false;
}

/**********************************************
 * Function: TestDrawColorImage
 * Description: for test, put every freespace contour into the map
 *              with different colors
 * Input: None
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
bool ContoursTracing::TestDrawColorImage() {
  for (int i = 0; i < rows_; i++) {
    for (int j = 0; j < cols_; j++) {
      color_image_.at<Vec3b>(i, j)[0] = 255;
      color_image_.at<Vec3b>(i, j)[1] = 255;
      color_image_.at<Vec3b>(i, j)[2] = 255;
    }
  }
  map<int, std::vector<cv::Point> >::iterator it;
  for (it = free_contours_.begin(); it != free_contours_.end(); it++) {
    for (int j = 0; j < it->second.size(); j++) {
      int dex = it->first * 2 / 255;
      int mdex = it->first * 2 % 255;
      if (dex == 0) {
        color_image_.at<Vec3b>(it->second[j])[0] = mdex;
        color_image_.at<Vec3b>(it->second[j])[1] = 0;
        color_image_.at<Vec3b>(it->second[j])[2] = 0;
      } else if (dex == 1) {
        color_image_.at<Vec3b>(it->second[j])[0] = 0;
        color_image_.at<Vec3b>(it->second[j])[1] = mdex;
        color_image_.at<Vec3b>(it->second[j])[2] = 0;
      } else if (dex == 2) {
        color_image_.at<Vec3b>(it->second[j])[0] = 0;
        color_image_.at<Vec3b>(it->second[j])[1] = 0;
        color_image_.at<Vec3b>(it->second[j])[2] = mdex;
      } else {
        color_image_.at<Vec3b>(it->second[j])[0] = 0;
        color_image_.at<Vec3b>(it->second[j])[1] = 0;
        color_image_.at<Vec3b>(it->second[j])[2] = 0;
      }
    }
  }
  namedWindow("color image", WINDOW_NORMAL);
  imshow("color image", color_image_);
  return true;
}

/**********************************************
 * Function: TestShowContourresult
 * Description: for test, show the result of the contour
 * Input: None
 * Output: None
 * Return: None
 * Others: None
 **********************************************/
bool ContoursTracing::TestShowContoursresult() {
  map<int, std::vector<cv::Point> >::iterator it;
  for (it = free_contours_.begin(); it != free_contours_.end(); it++) {
    TestContourImage(it->first, 1);
    std::string initname("free");
    std::string curname = initname + std::to_string(it->first);
  }
  ShowSingleContour("free");
  // for (it = obs_contours_.begin(); it != obs_contours_.end(); it++) {
  //   TestContourImage(it->first, 0);
  //   std::string initname("obs");
  //   std::string curname = initname + std::to_string(it->first);
  //   ShowSingleContour(curname);
  // }
  return true;
}

/**********************************************
 * Function: TestContoursDetection
 * Description: for test, using the binary_image_clone_test_ to create the
 *contours Input: None Output: None Return: None Others: None
 **********************************************/
bool ContoursTracing::TestContoursDetection() {
  if (!binary_flag_) {
    std::cout << "binarize failed" << std::endl;
    return false;
  }

  std::vector<std::vector<Point> > contours;
  std::vector<Vec4i> hierarchy;
  findContours(binary_image_clone_test_, contours, hierarchy, CV_RETR_CCOMP,
               CV_CHAIN_APPROX_SIMPLE);
  std::cout << "number of the test image:" << contours.size() << std::endl;
  contours_.clear();
  for (int i = 0; i < contours.size(); i++) {
    std::cout << contours[i].size() << std::endl;
    // if (contours[i].size()>10) {
    //   contours_.push_back(contours[i]);
    // }
    // Rect rect = boundingRect(contours[i]);//检测外轮廓
    // rectangle(binary_image_clone_test_, rect, cv::Scalar::all(255),
    // 0);//对外轮廓加矩形框
  }
  // contours_.push_back(contours[2]);
  drawContours(binary_image_clone_test_, contours, -1, cv::Scalar::all(255));
  return true;
}

bool ContoursTracing::TestContourImage(const int& index, const bool& freeflag) {
  ClearContourMat();
  output_file.open("output.txt", ofstream::out);
  if (freeflag) {
    for (int i = 0; i < free_contours_[index].size(); i++) {
      single_contour_mat_.at<uchar>(free_contours_[index][i]) = 255;
      output_file << "free" << index << ":" << free_contours_[index][i].x << ","
                  << free_contours_[index][i].y << endl;
    }
  } else {
    for (int i = 0; i < obs_contours_[index].size(); i++) {
      single_contour_mat_.at<uchar>(obs_contours_[index][i]) = 255;
      output_file << "obs" << index << ":" << obs_contours_[index][i].x << ","
                  << obs_contours_[index][i].y << endl;
    }
  }
  std::map<int, std::vector<cv::Point> >::iterator it;
  for (it = obs_contours_.begin(); it != obs_contours_.end(); it++) {
    cout << "obs" << index << "1:" << it->first << ",2:" << it->second.size()
         << endl;
  }
  for (it = free_contours_.begin(); it != free_contours_.end(); it++) {
    cout << "free" << index << "1:" << it->first << ",2:" << it->second.size()
         << endl;
  }
  output_file.close();
  return true;
}
