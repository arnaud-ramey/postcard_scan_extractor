/*!
  \file        postcard_scan_extractor.h
  \author      Arnaud Ramey <arnaud.a.ramey@gmail.com>
                -- Robotics Lab, University Carlos III of Madrid
  \date        2014/11/15

________________________________________________________________________________

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU Lesser General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
Lesser General Public License for more details.

You should have received a copy of the GNU Lesser General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
________________________________________________________________________________

\todo Description of the file
 */

#ifndef postcard_scan_extractor_H
#define postcard_scan_extractor_H

#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <stdio.h>
#include <iostream>
#include "postcard_scan_extractor_path.h"

#define DEBUG_PRINT(...)   {}
//#define DEBUG_PRINT(...)   printf(__VA_ARGS__)

////////////////////////////////////////////////////////////////////////////////

/*!
 * Remove the extension from a filename
 * \param path
 *  the full path
 * \example
 *  "/foo/bar" -> "/foo/bar"
 *  "/foo/bar.dat" -> "/foo/bar"
 *  "/foo.zim/bar.dat" -> "/foo.zim/bar"
 *  "/foo.zim/bar" -> "/foo.zim/bar"
 */
inline std::string remove_filename_extension(const std::string & path) {
  std::string::size_type dot_pos = path.find_last_of('.');
  if (dot_pos == std::string::npos)
    return path;
  std::string::size_type slash_pos = path.find_last_of('/');
  if (slash_pos != std::string::npos && slash_pos > dot_pos) // dot before slash
    return path;
  return path.substr(0, dot_pos);
}

//! \brief   returns |ab|, L2 norm
template<class _Pt2>
inline double dist_L2(const _Pt2 & a, const _Pt2 & b) {
  return hypot(a.x - b.x, a.y - b.y);
}

////////////////////////////////////////////////////////////////////////////////

class PostcardScanExtractor {
public:
  static const unsigned int WINDOW_WIDTH = 800, WINDOW_HEIGHT = 600, MARGIN_SIZE = 10;
  static const unsigned int ZOOM_WINDOW_SIZE  = 200;
  static const double DEFAULT_ZOOM_LEVEL = 50;

  struct Line {
    static Line perpendicular_at_point(const cv::Point & A,
                                       const cv::Point & B) {
      Line line;
      line._A = A;
      line._vec[0] = A.y - B.y;
      line._vec[1] = B.x - A.x;
      return line;
    }
    void draw(cv::Mat & img, const cv::Scalar& color,
              int thickness=1, int lineType=8, int shift=0) {
      if (fabs(_vec[0]) < 1E-3) // vertical line
        cv::line(img, cv::Point(_A.x, 0), cv::Point(_A.x, img.rows),
                 color, thickness, lineType, shift);
      else if (fabs(_vec[1]) < 1E-3) // horizontal line
        cv::line(img, cv::Point(0, _A.y), cv::Point(img.cols, _A.x),
                 color, thickness, lineType, shift);
      else {
        cv::line(img, cv::Point(_A.x, 0), cv::Point(_A.x, img.rows),
                 color, thickness, lineType, shift);
      }
    }

    cv::Point _A;
    cv::Vec2i _vec;
  }; ///////////////////////////////////////////////////////////////////////////

  PostcardScanExtractor(const std::string & postcard_suffix = "_postcard") :
    MAIN_WINDOW_NAME("PostcardScanExtractor"),
    _postcard_suffix(postcard_suffix)
  {
    DEBUG_PRINT("ctor\n");
    // declare window
    cv::namedWindow(MAIN_WINDOW_NAME);
    cv::setMouseCallback(MAIN_WINDOW_NAME, PostcardScanExtractor::win_cb, this);
    // create default images
    _scan_img_hires.create(WINDOW_HEIGHT, WINDOW_WIDTH);
    _scan_img_hires.setTo(cv::Scalar::all(255));
    _zoom_img.create(ZOOM_WINDOW_SIZE, ZOOM_WINDOW_SIZE);
    _zoom_level = DEFAULT_ZOOM_LEVEL;
    _last_mouse_x = _last_mouse_y = 0;
    _cross_color1 = CV_RGB(255, 0, 0);
    _cross_color2 = CV_RGB(0, 0, 0);
    // playlist
    _playlist_idx = 0;
    set_image(_scan_img_hires.clone());
  } // end ctor

  //////////////////////////////////////////////////////////////////////////////

  inline bool load_playlist_images(const std::vector<std::string> & playlist) {
    DEBUG_PRINT("load_playlist_images(%i images)\n", playlist.size());
    bool contains_help = (std::find(playlist.begin(), playlist.end(), std::string("--help"))
                          != playlist.end())
        ||
        (std::find(playlist.begin(), playlist.end(), std::string("-h"))
         != playlist.end());
    if (playlist.empty() || contains_help) {
      // printf("Cannot load an empty playlist! Exiting.\n");
      // print help
      std::cout << "Synopsis" << std::endl
                << "  postcard_scan_extractor INPUTFILES" << std::endl
                << "Description" << std::endl
                << "  INPUTFILES  can be any file read by OpenCV, notably JPEGs, PNGs, BMPs, etc." << std::endl
                << std::endl
                << "Keys:" << std::endl
                << "  MOUSE MOVE:             move cursor" << std::endl
                << "  UP, DOWN, LEFT, RIGHT:  precisely move cursor" << std::endl
                << "  LEFT CLICK, ENTER:      set current cursor position as center" << std::endl
                << "  '+':                    increase zoom level" << std::endl
                << "  '-':                    decrease zoom level" << std::endl
                << "  'p', BACKSPACE:         go to previous image" << std::endl
                << "  'n', SPACE:             go to next image" << std::endl
                << "  'r':                    rotate last postcard of 90°" << std::endl
                << "  'f','v':                flip last postcard vertically" << std::endl
                << "  'h':                    flip last postcard horizontally" << std::endl
                << "  'q', ESCAPE:            exit program" << std::endl;
      quit();
    }
    _playlist = playlist;
    return goto_playlist_image(0);
  }

  //////////////////////////////////////////////////////////////////////////////

  inline bool goto_next_playlist_image() {
    unsigned int image_idx = (_playlist_idx + 1) % _playlist.size();
    return goto_playlist_image(image_idx);
  }
  inline bool goto_prev_playlist_image() {
    unsigned int image_idx = (_playlist_idx + _playlist.size() - 1) % _playlist.size();
    return goto_playlist_image(image_idx);
  }
  inline bool goto_playlist_image(unsigned int playlist_idx) {
    if (playlist_idx < 0 || playlist_idx >= _playlist.size())
      return false;
    _playlist_idx = playlist_idx;
    return load_playlist_image(get_current_filename());
  }

  //////////////////////////////////////////////////////////////////////////////

  inline void run() {
    while(true) {
      cv::imshow(MAIN_WINDOW_NAME, _final_window);
      char c = cv::waitKey(50);
      int i = (int) c;
      //DEBUG_PRINT("c:%c = %i\n", c, i);

      if (c == 'p' || c == 8) // || c == 81 || i == 85) // 81:left | 85:PageUp | 8:backspace
        goto_prev_playlist_image();
      else if (c == 'n' || c == ' ') // || c == 83 || i == 86) // 83:right | 86:PageDown
        goto_next_playlist_image();

      else if (i == 81) { // left
        _last_mouse_x -= .5;
        recompute_zoom_and_redraw();
      }
      else if (i == 83) { // right
        _last_mouse_x += .5;
        recompute_zoom_and_redraw();
      }
      else if (i == 82) { // up
        _last_mouse_y -= .5;
        recompute_zoom_and_redraw();
      }
      else if (i == 84) { // down
        _last_mouse_y += .5;
        recompute_zoom_and_redraw();
      }
      else if (c == '+' || i == -85) { // -85:'+' keypad
        _zoom_level = cv::max(_zoom_level - 1, 1.);
        recompute_zoom_and_redraw();
      }
      else if (c == '-' || i == -83) { // -83:'-' keypad
        ++_zoom_level;
        recompute_zoom_and_redraw();
      }
      else if (i == 10 || i == -115) { // return | keypad return
        add_corner(_last_mouse_x, _last_mouse_y);
      }
      else if (c == 'r' && !_postcard.empty()) { // rotate postcard 90°
        cv::transpose(_postcard, _postcard_buffer);
        cv::flip(_postcard_buffer, _postcard, 0);
        save_current_postcard();
      }
      else if ((c == 'f'|| c=='v') && !_postcard.empty()) { // flip postcard
        cv::flip(_postcard, _postcard, 1); // y axis
        save_current_postcard();
      }
      else if (c == 'h' && !_postcard.empty()) { // flip postcard
        cv::flip(_postcard, _postcard, 0); // x axis
        save_current_postcard();
      }
      else if (c == 27 || c == 'q') {
        quit();
        break;
      }
    } //end while (true)
  } // end run()

  //////////////////////////////////////////////////////////////////////////////

protected:

  //////////////////////////////////////////////////////////////////////////////

  static void win_cb(int event, int x, int y, int flags, void* cookie) {
    // DEBUG_PRINT("win_cb(%i, %i): event:%i, flag:%i\n", x, y, event, flags);
    PostcardScanExtractor* this_cb = ((PostcardScanExtractor*) cookie);
    if (event == CV_EVENT_LBUTTONDOWN) // do not move mouse if left click
      this_cb->add_corner(this_cb->_last_mouse_x, this_cb->_last_mouse_y);
    else {
      if (this_cb->_gui_corners.size() == 2) { // only allow perpendicular third point
        double x1 = this_cb->_gui_corners.front().x, y1 = this_cb->_gui_corners.front().y;
        double x2 = this_cb->_gui_corners.back().x, y2 = this_cb->_gui_corners.back().y;
        double a = x2 - x1, b = y2 - y1;
        cv::Mat A = (cv::Mat_<double>(4,4)
                     << b, 0, 1, 0,
                     a, 0, 0, -1,
                     0, a, -1, 0,
                     0, b, 0, -1);
        cv::Mat B = (cv::Mat_<double>(4,1) << x2, -y2, -x, -y);
        cv::Mat AlphaBetaX3Y3 = A.inv() * B;
        x = AlphaBetaX3Y3.at<double>(2);
        y = AlphaBetaX3Y3.at<double>(3);
      }
      this_cb->_last_mouse_x = x;
      this_cb->_last_mouse_y = y;
    }
    if (event == CV_EVENT_RBUTTONDOWN) { // clear on right button
      this_cb->_gui_corners.clear();
      this_cb->_hires_corners.clear();
    }
    this_cb->recompute_zoom_and_redraw();
  } // end win_cb();

  //////////////////////////////////////////////////////////////////////////////

  inline void add_corner(int x, int y) {
    DEBUG_PRINT("adding corner(%i, %i)\n", x, y);
    if (_gui_corners.size() >= 3) { // clear corners if new postcard
      _gui_corners.clear();
      _hires_corners.clear();
      ++_postcard_idx; // must be after save_current_postcard()
    }
    _gui_corners.push_back(cv::Point(x, y));
    _hires_corners.push_back(cv::Point2f((1. * x - MARGIN_SIZE) / _big2small_factor,
                                         (1. * y - MARGIN_SIZE) / _big2small_factor));
    if (_gui_corners.size() < 3) { // do nothing if corners not over
      redraw_final_window();
      return;
    }
    // add fourth corner
    _gui_corners.push_back(_gui_corners[0] + _gui_corners[2] - _gui_corners[1]);
    // compute postcard
    double w = dist_L2(_hires_corners[0], _hires_corners[1]);
    double h = dist_L2(_hires_corners[1], _hires_corners[2]);
    cv::Point2f src [3]= { _hires_corners[0], _hires_corners[1], _hires_corners[2]};
    cv::Point2f dst [3]= { cv::Point2f(0, 0), cv::Point2f(w, 0), cv::Point2f(w, h)};
    cv::Mat transform = cv::getAffineTransform(src, dst);
    _postcard.create(h, w);
    _postcard.setTo(cv::Scalar::all(0));
    cv::warpAffine(_scan_img_hires, _postcard, transform,  _postcard.size());
    save_current_postcard(); // also refresh thumbnail
  }

  //////////////////////////////////////////////////////////////////////////////

  inline virtual bool load_playlist_image(const std::string & filename) {
    DEBUG_PRINT("load_playlist_image('%s')\n", filename.c_str());
    cv::Mat3b scan_img = cv::imread(filename, CV_LOAD_IMAGE_COLOR);
    if (scan_img.empty()) {
      printf("Could not read file '%s'!\n", filename.c_str());
      return false;
    }
    return set_image(scan_img);
  }
  inline std::string get_current_filename() const {
    return _playlist[_playlist_idx];
  }
  inline std::string get_current_postcard_filename() const {
    std::ostringstream ans;
    ans << remove_filename_extension(get_current_filename())
        << _postcard_suffix << _postcard_idx << ".png";
    return ans.str();
  }

  //////////////////////////////////////////////////////////////////////////////

  bool set_image(const cv::Mat3b & scan_img_hires) {
    DEBUG_PRINT("set_images(scan_img:%ix%i)\n",
                scan_img_hires.cols, scan_img_hires.rows);
    if (scan_img_hires.empty()) {
      printf("Cannot set an empty contour image!\n");
      return false;
    }
    // rotate 90° if needed
    if (scan_img_hires.cols < scan_img_hires.rows) {
      cv::transpose(scan_img_hires, _scan_img_hires);
      cv::flip(_scan_img_hires, _scan_img_hires, 0);
    }
    else { // no rotation
      scan_img_hires.copyTo(_scan_img_hires);
    }
    // resize
    _big2small_factor = cv::min(1. * WINDOW_WIDTH / _scan_img_hires.cols,
                                1. * WINDOW_HEIGHT / _scan_img_hires.rows);
    cv::resize(_scan_img_hires, _scan_img_lores, cv::Size(), _big2small_factor, _big2small_factor);
    // prepair _final_window
    unsigned int scan_margin_cols = _scan_img_lores.cols + 2 * MARGIN_SIZE;
    unsigned int cols = scan_margin_cols + ZOOM_WINDOW_SIZE;
    unsigned int rows = std::max(_scan_img_lores.rows + 2 * MARGIN_SIZE,
                                 2 * ZOOM_WINDOW_SIZE + MARGIN_SIZE); // zoom + postcard thumbnail
    _final_window.create(rows, cols);
    _final_window.setTo(cv::Scalar::all(100));
    cv::Rect zoom_roi(scan_margin_cols, 0, ZOOM_WINDOW_SIZE, ZOOM_WINDOW_SIZE);
    _final_zoom_img = _final_window(zoom_roi);
    cv::Rect scan_roi(MARGIN_SIZE, MARGIN_SIZE, _scan_img_lores.cols, _scan_img_lores.rows);
    _final_scan_img = _final_window(scan_roi);
    // clear data
    _postcard_idx = 0;
    _gui_corners.clear();
    _hires_corners.clear();
    _postcard.release();
    _postcard_thumbnail.release();
    recompute_zoom_and_redraw();
    return true;
  } // end set_image()

  //////////////////////////////////////////////////////////////////////////////

  inline void quit() {
    printf("The application will shut down now. Have a nice day.\n");
    exit(0);
  } // end quit()

  //////////////////////////////////////////////////////////////////////////////

  void redraw_final_window() {
    DEBUG_PRINT("redraw_final_window(%g,%g)\n", _last_mouse_x, _last_mouse_y);
    // copy scan
    _scan_img_lores.copyTo(_final_scan_img);
    // draw main window cross
    draw_bicolor_cross(_final_scan_img, _last_mouse_x-MARGIN_SIZE,
                       _last_mouse_y-MARGIN_SIZE, _cross_color1, _cross_color2);
    // draw postcard corners
    unsigned int ncorners = _gui_corners.size();
    for(unsigned int i = 0; i < ncorners; ++i) {
      cv::circle(_final_window, _gui_corners[i], 3, CV_RGB(0, 0, 255), 2);
      if (i < ncorners - 1 || ncorners == 4)
        cv::line(_final_window, _gui_corners[i],  _gui_corners[(i+1)%4],
            CV_RGB(0, 0, 255), 1);
    }
    // redraw_zoom_window
    _zoom_img.copyTo(_final_zoom_img);
    // draw zoom cross
    draw_bicolor_cross(_final_zoom_img, ZOOM_WINDOW_SIZE/2,ZOOM_WINDOW_SIZE/2,
                       _cross_color1, _cross_color2);
    // draw postcard thumbnail
    if (_postcard_thumbnail.empty())
      return;
    _postcard_thumbnail.copyTo(_final_postcard_img);
  } // end redraw_final_window();

  //////////////////////////////////////////////////////////////////////////////

  static const inline void draw_bicolor_cross(cv::Mat3b & img,
                                              const unsigned int x,  const unsigned int y,
                                              cv::Scalar color1, cv::Scalar color2,
                                              unsigned int thickness = 1) {
    static const unsigned int step = 10;
    unsigned int ncols = img.cols, nrows = img.rows;
    for(unsigned int col = 0; col < ncols; col+=step) {
      cv::line(img, cv::Point(col, y), cv::Point(col + step, y), color1, thickness);
      std::swap(color1, color2);
    }
    for(unsigned int row = 0; row < nrows; row+=step) {
      cv::line(img, cv::Point(x, row), cv::Point(x, row + step), color1, thickness);
      std::swap(color1, color2);
    }
  } // end draw_bicolor_cross();

  //////////////////////////////////////////////////////////////////////////////

  bool recompute_zoom_and_redraw() {
    double x_hires = (_last_mouse_x - MARGIN_SIZE) / _big2small_factor,
        y_hires = (_last_mouse_y - MARGIN_SIZE) / _big2small_factor;
    //DEBUG_PRINT("(%g,%g) ->redraw zoom at (%i,%i)\n", _last_mouse_x, _last_mouse_y, x_hires, y_hires);
    double zoom_size = _zoom_level / _big2small_factor;
    cv::Point2f src [3]= { cv::Point2f(x_hires - zoom_size, y_hires - zoom_size),
                           cv::Point2f(x_hires + zoom_size, y_hires - zoom_size),
                           cv::Point2f(x_hires + zoom_size, y_hires + zoom_size)};
    cv::Point2f dst [3]= { cv::Point2f(0, 0),
                           cv::Point2f(ZOOM_WINDOW_SIZE, 0),
                           cv::Point2f(ZOOM_WINDOW_SIZE, ZOOM_WINDOW_SIZE)};
    cv::Mat transform = cv::getAffineTransform(src, dst);
    cv::warpAffine(_scan_img_hires, _zoom_img, transform,  _zoom_img.size());
    redraw_final_window();
    return true;
  } // end recompute_zoom_and_redraw()

  //////////////////////////////////////////////////////////////////////////////

  inline bool save_current_postcard() {
    DEBUG_PRINT("save_current_postcard('%s')\n", get_current_postcard_filename().c_str());
    if (_postcard.empty())
      return false;
    // recompute postcard thumbnail
    double ps2thumb_factor = cv::min(1. * ZOOM_WINDOW_SIZE / _postcard.cols,
                                     1. * ZOOM_WINDOW_SIZE / _postcard.rows);
    cv::resize(_postcard, _postcard_thumbnail, cv::Size(),
               ps2thumb_factor, ps2thumb_factor);
    unsigned int scan_margin_cols = _scan_img_lores.cols + 2 * MARGIN_SIZE;
    cv::Rect _postcard_roi (scan_margin_cols, ZOOM_WINDOW_SIZE + MARGIN_SIZE,
                            _postcard_thumbnail.cols,  _postcard_thumbnail.rows);
    _final_postcard_img = _final_window(_postcard_roi);
    redraw_final_window(); // refresh thumb

    // save postcard
    if (!cv::imwrite(get_current_postcard_filename(), _postcard)) {
      printf("Could not write '%s'!\n", get_current_postcard_filename().c_str());
      return false;
    }
    printf("Succesfully written '%s'\n", get_current_postcard_filename().c_str());
    return true;
  } // end save_current_postcard()

  //////////////////////////////////////////////////////////////////////////////

  // GUI
  std::string MAIN_WINDOW_NAME;
  double _last_mouse_x, _last_mouse_y, _big2small_factor, _zoom_level;
  cv::Mat3b _scan_img_hires, _scan_img_lores, _zoom_img, _final_window, _postcard;
  cv::Scalar _cross_color1, _cross_color2;
  cv::Mat3b _final_zoom_img, _final_postcard_img, _final_scan_img;
  // postcard
  cv::Mat3b _postcard_buffer, _postcard_thumbnail;
  std::string _postcard_suffix;
  std::vector<cv::Point> _gui_corners;
  std::vector<cv::Point2f> _hires_corners;
  unsigned int _postcard_idx;
  // playlist
  std::vector<std::string> _playlist;
  unsigned int _playlist_idx;
}; // en class PostcardScanExtractor

#endif // postcard_scan_extractor_H
