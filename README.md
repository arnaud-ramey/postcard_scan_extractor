                  +-------------------------+
                  | postcard_scan_extractor |
                  +-------------------------+

[![Build Status](https://travis-ci.org/arnaud-ramey/postcard_scan_extractor.svg)](https://travis-ci.org/arnaud-ramey/postcard_scan_extractor)

A Graphical User Interface for extracting postcards from a scanned image.
It is made for being fast and handy.
For instance, the extracted postcards can be rotated from the interface itself.

License :                  see the LICENSE file.
Authors :                  see the AUTHORS file.
How to build the program:  see the INSTALL file.

________________________________________________________________________________

How to use the program
________________________________________________________________________________
To display the help, just launch the program in a terminal.
It will display the help of the program.

Synopsis
  postcard_scan_extractor INPUTFILES
Description
  INPUTFILES  can be any file read by OpenCV, notably JPEGs, PNGs, BMPs, etc.

Keys:
  MOUSE MOVE:             move cursor
  UP, DOWN, LEFT, RIGHT:  precisely move cursor
  LEFT CLICK, ENTER:      set current cursor position as center
  '+':                    increase zoom level
  '-':                    decrease zoom level
  'p', BACKSPACE:         go to previous image
  'n', SPACE:             go to next image
  'r':                    rotate last postcard of 90°
  'f','v':                flip last postcard vertically
  'h':                    flip last postcard horizontally
  'q', ESCAPE:            exit program

