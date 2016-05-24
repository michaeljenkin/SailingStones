!/usr/bin/env python
#
# Software License Agreement (BSD License)
#
# Copyright (c) 2009, Willow Garage, Inc.
00006 # All rights reserved.
00007 #
00008 # Redistribution and use in source and binary forms, with or without
00009 # modification, are permitted provided that the following conditions
00010 # are met:
00011 #
00012 #  * Redistributions of source code must retain the above copyright
00013 #    notice, this list of conditions and the following disclaimer.
00014 #  * Redistributions in binary form must reproduce the above
00015 #    copyright notice, this list of conditions and the following
00016 #    disclaimer in the documentation and/or other materials provided
00017 #    with the distribution.
00018 #  * Neither the name of the Willow Garage nor the names of its
00019 #    contributors may be used to endorse or promote products derived
00020 #    from this software without specific prior written permission.
00021 #
00022 # THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
00023 # "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
00024 # LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
00025 # FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
00026 # COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
00027 # INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
00028 # BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
00029 # LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
00030 # CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
00031 # LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
00032 # ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
00033 # POSSIBILITY OF SUCH DAMAGE.
00034 
00035 import rosunit
00036 import subprocess
00037 import unittest
00038 
00039 class TestParser(unittest.TestCase):
00040     def test_ini(self):
00041         for files in [('calib5.ini', 'calib5.yaml'), ('calib8.ini', 'calib8.yaml')]:
00042             for dir in [ '', './']:
00043                 p = subprocess.Popen('rosrun camera_calibration_parsers convert $(rospack find camera_calibration_parsers)/test/%s %s%s' % (files[0], dir, files[1]), shell=True, stderr=subprocess.PIPE)
00044                 out, err = p.communicate()
00045                 self.assertEqual(err, '')
00046 
00047 if __name__ == '__main__':
00048     rosunit.unitrun('camera_calibration_parsers', 'parser', TestParser)
