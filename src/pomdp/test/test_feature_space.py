#!/usr/bin/env python

# The MIT License (MIT)
# 
# Copyright (c) 2014 Boone "Bea" Adkins
# 
# Permission is hereby granted, free of charge, to any person obtaining a copy
# of this software and associated documentation files (the "Software"), to deal
# in the Software without restriction, including without limitation the rights
# to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
# copies of the Software, and to permit persons to whom the Software is
# furnished to do so, subject to the following conditions:
# 
# The above copyright notice and this permission notice shall be included in all
# copies or substantial portions of the Software.
# 
# THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
# IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
# FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
# AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
# LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
# OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
# SOFTWARE.

PKG='POMDP'

from nose.tools import *

from pomdp.feature_space import *

# Classes test valid initialization in setupClass method. Test invalid initialization in own tests.


fs_color_param = {'name':'color', 'values':['red', 'blue', 'yellow']}                
class TestSimpleFeatureSpaceNames():   
    def setUp(self):
        self.fs = SimpleFeatureSpace(fs_color_param)
        
    def test_init_too_short(self):
        param = {'name':'colorless', 'values':['black']}
        assert_raises(ValueError, SimpleFeatureSpace, param)

    def test_num_values(self):
        assert_equal(self.fs.num_values(), 3)

    def test_is_valid_valid(self):
        assert_true(self.fs.is_valid("red"))
        assert_true(self.fs.is_valid("blue"))
        assert_true(self.fs.is_valid("yellow"))
        assert_true(self.fs.is_valid(0))
        assert_true(self.fs.is_valid(2))
        
    def test_is_valid_out_of_range(self):
        assert_false(self.fs.is_valid("purple"))
        assert_false(self.fs.is_valid("green"))
        assert_false(self.fs.is_valid(-1))
        assert_false(self.fs.is_valid(3))

    def test_is_valid_invalid(self):                         
        assert_false(self.fs.is_valid(("red", 2)))
        assert_false(self.fs.is_valid([4, 3]))

fs_num_vals_param = {'name':'x', 'values':9}
class TestSimpleFeatureSpaceNumVals():
    def setUp(self):
        self.fs = SimpleFeatureSpace(fs_num_vals_param)

    def test_init_too_short(self):         
        param = {'name':'ynt-sized', 'values':1}
        assert_raises(ValueError, SimpleFeatureSpace, param)

    def test_init_too_negative(self):             
        param = {'name':'zany', 'values':-499}
        assert_raises(ValueError, SimpleFeatureSpace, param)

    def test_num_values(self):
        assert_equal(self.fs.num_values(), 9)

    def test_is_valid_valid(self):
        assert_true(self.fs.is_valid(0))
        assert_true(self.fs.is_valid(8))
        
    def test_is_valid_out_of_range(self):
        assert_false(self.fs.is_valid(-1))
        assert_false(self.fs.is_valid(9))

    def test_is_valid_invalid(self):
        assert_false(self.fs.is_valid("one"))                         
        assert_false(self.fs.is_valid(("red", 2)))
        assert_false(self.fs.is_valid([4, 3]))

fs_coord_param = [
                  {'name':'x', 'values': 9},
                  {'name':'y', 'values': 12}
                 ]
class TestVectorFeatureSpace():
    def setUp(self):
        self.fs = VectorFeatureSpace(fs_coord_param)
        
    def test_init_feature_vector_valid(self):
        fs_vector = VectorFeatureSpace(fs_coord_param)
   
    def test_is_valid_valid(self):
        assert_true(self.fs.is_valid((0,11)))
        assert_true(self.fs.is_valid([8, 0]))
        assert_true(self.fs.is_valid(0))
        assert_true(self.fs.is_valid(107))
        
    def test_is_valid_out_of_range(self):
        assert_false(self.fs.is_valid([0,12]))
        assert_false(self.fs.is_valid((-1,11)))
        assert_false(self.fs.is_valid(-1))
        assert_false(self.fs.is_valid(108))

    def test_is_valid_invalid(self):                         
        assert_false(self.fs.is_valid("red"))
        assert_false(self.fs.is_valid([4, 3, 4]))

    def test_num_values(self):
        assert_equal(self.fs.num_values(), 108)

    def test_hash(self):
        assert_equal(self.fs.hash([0, 0]), 0)        
        assert_equal(self.fs.hash([8, 11]), 107)

    def test_dehash(self):
        assert_equal(self.fs.dehash(0), [0, 0])        
        assert_equal(self.fs.dehash(107), [8, 11])


fs_pixel_param = [
                  {'name':'x', 'values':3},
                  {'name':'y', 'values':2},
                  {'name':'color', 'values':['red', 'blue', 'green', 'yellow']}
                 ]
class TestVectorFeatureSpace2():
    def setUp(self):
        self.fs = VectorFeatureSpace(fs_pixel_param)
        
    def test_init_feature_vector_valid(self):
        fs_vector = VectorFeatureSpace(fs_pixel_param)

    def test_num_values(self):
        assert_equal(self.fs.num_values(), 24)
        
    def test_is_valid_valid(self):
        assert_true(self.fs.is_valid((1, 1, 1)))
        assert_true(self.fs.is_valid(0))
        assert_true(self.fs.is_valid(23))
        
    def test_is_valid_out_of_range(self):
        assert_false(self.fs.is_valid([0, 0, 4]))
        assert_false(self.fs.is_valid([0, 3, 0]))
        assert_false(self.fs.is_valid(-1))
        assert_false(self.fs.is_valid((24)))

    def test_is_valid_invalid(self):                         
        assert_false(self.fs.is_valid("red"))
        assert_false(self.fs.is_valid([0, 0]))
        assert_false(self.fs.is_valid([0, 0, 1, 0]))

    def test_hash(self):
        assert_equal(self.fs.hash([0, 0, 0]), 0)
        assert_equal(self.fs.hash([2, 1, 1]), 11)
        assert_equal(self.fs.hash([0, 1, 2]), 15)
        assert_equal(self.fs.hash([2, 1, 3]), 23)

    def test_dehash(self):
        assert_equal(self.fs.dehash(0), [0, 0, 0])
        assert_equal(self.fs.dehash(11), [2, 1, 1])
        assert_equal(self.fs.dehash(15), [0, 1, 2])
        assert_equal(self.fs.dehash(23), [2, 1, 3])