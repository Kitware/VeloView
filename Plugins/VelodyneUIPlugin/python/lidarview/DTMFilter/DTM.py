#!/usr/bin/env python
# -*- coding: utf-8 -*-

###############################################################################
# Copyright Kitware Inc. and Contributors
# Distributed under the Apache License, 2.0 (apache.org/licenses/LICENSE-2.0)
# See accompanying Copyright.txt and LICENSE files for details
###############################################################################

# source: Danesfield project
# link: https://github.com/Kitware/Danesfield/blob/master/danesfield/dtm.py

# modifications: only prints, to run from Python2
from __future__ import print_function


"""DTM (Digital Terrain Model) estimation from a DSM (Digital Surface Model)
"""

import numpy
import scipy.ndimage as ndimage


class DTMEstimator(object):
    """A class to estimate a DTM from a DSM
    """

    def __init__(self, nodata_val=-9999, num_outer_iter=100,
                 num_inner_iter=10, base_step=1):
        """Constructor
        """
        if nodata_val is None:
            nodata_val = -9999
        self.nodata_val = nodata_val
        self.num_outer_iter = num_outer_iter
        self.num_inner_iter = num_inner_iter
        self.base_step = base_step

    @staticmethod
    def downsample(dtm):
        """Simple 2X downsampling, take every other pixel
        """
        return dtm[::2, ::2]

    @staticmethod
    def upsample(dtm, out):
        """Simple 2X upsampling, duplicate pixels
        """
        # Adjust the slicing for odd row count
        if out.shape[0] % 2 == 1:
            s0 = numpy.s_[:-1]
        else:
            s0 = numpy.s_[:]
        # Adjust the slicing for odd column count
        if out.shape[1] % 2 == 1:
            s1 = numpy.s_[:-1]
        else:
            s1 = numpy.s_[:]

        # copy in duplicate values for blocks of 2x2 pixels
        out[::2, ::2] = dtm
        out[1::2, ::2] = dtm[s0, :]
        out[::2, 1::2] = dtm[:, s1]
        out[1::2, 1::2] = dtm[s0, s1]

    def recursive_fit_dtm(self, dtm, dsm, step=1, level=0):
        """
        Recursive function to apply multi-scale DTM fitting
        """
        # if the image is still larger than 100 pixels, downsample
        if numpy.min(dtm.shape) > 100:
            # downsample both the DTM and DSM
            sm_dtm = self.downsample(dtm)
            sm_dsm = self.downsample(dsm)
            # Recursively apply DTM fitting to the downsampled image
            sm_dtm, max_level = self.recursive_fit_dtm(sm_dtm, sm_dsm, step, level+1)
            # Upsample the DTM back to the original resolution
            self.upsample(sm_dtm, dtm)
            print("level {} of {}".format(level, max_level))
            # Decrease the step size exponentially when moving back down the pyramid
            step = step / (2 * 2 ** (max_level - level))
            # Decrease the number of iterations as well
            num_iter = max(1, int(self.num_outer_iter / (2 ** (max_level - level))))
            # Apply iterations of cloth draping simulation to smooth out the result
            return self.drape_cloth(dtm, dsm, step, num_iter), max_level

        print("reached min size {}".format(dtm.shape))
        # Apply cloth draping at the coarsest level (base case)
        return self.drape_cloth(dtm, dsm, step, self.num_outer_iter), level

    def drape_cloth(self, dtm, dsm, step=1, num_outer_iter=10):
        """
        Compute inverted 2.5D cloth draping simulation iterations
        """
        print("draping:", end='')
        for i in range(num_outer_iter):
            # print(".", end='', flush=True)
            print(".", end='') # for python2
            # raise the DTM by step (inverted gravity)
            valid = dsm != self.nodata_val
            dtm[valid] += step
            for i in range(self.num_inner_iter):
                # handle DSM intersections, snap back to below DSM
                numpy.minimum(dtm, dsm, out=dtm, where=valid)
                # apply spring tension forces (blur the DTM)
                dtm = ndimage.uniform_filter(dtm, size=3)
        # print newline after progress bar
        print("")
        # one final intersection check
        numpy.minimum(dtm, dsm, out=dtm, where=valid)
        return dtm

    def fit_dtm(self, dsm):
        """
        Fit a Digital Terrain Model (DTM) to the provided Digital Surface Model (DSM)
        """
        # initialize DTM to a deep copy of the DSM
        dtm = dsm.copy()
        # get the range of valid height values (skipping no-data values)
        valid_data = dsm[dsm != self.nodata_val]
        minv = numpy.min(valid_data)
        maxv = numpy.max(valid_data)
        # compute the step size that covers the range in num_iter steps
        step = (maxv - minv) / self.num_outer_iter
        # initialize the DTM values to the minimum DSM height
        dtm = numpy.full(dsm.shape, minv, dsm.dtype)
        return self.recursive_fit_dtm(dtm, dsm, step)[0] 
