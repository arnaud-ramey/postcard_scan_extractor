/*!
  \file        postcard_scan_extractor.cpp
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

A simple GUI for PostcardScanExtractors.
 */

#include "postcard_scan_extractor.h"

int main(int argc, char** argv) {
  std::vector<std::string> filenames;
#if 0
  filenames.push_back(POSTCARD_SCAN_EXTRACTOR_PATH "samples/sample2.jpg");
  filenames.push_back(POSTCARD_SCAN_EXTRACTOR_PATH "samples/sample1.jpg");
  filenames.push_back(POSTCARD_SCAN_EXTRACTOR_PATH "samples/sample3.jpg");
  PostcardScanExtractor annot;
#else
  for (unsigned int i = 1; i < argc; ++i)
    filenames.push_back(std::string(argv[i]));
  PostcardScanExtractor annot;
#endif
  annot.load_playlist_images(filenames);
  annot.run();
}
