import numpy as np
import vtk
import ctypes as C
import DTM

nodata_val = 0.0
print("Interpreting value {} as \"no data available\"".format(nodata_val))
estimator = DTM.DTMEstimator(nodata_val=nodata_val)

# inputImage is expected to be vtkImageData !
inputImage = self.GetInput()
outputImage = self.GetOutput()

outputImage.DeepCopy(inputImage)
# should you want to modify without casting to numpy arra:
# outputImage.SetScalarComponentFromDouble(i, j, 0, 0, 0.5) # works: ouput of filter is different on PV screen

dims = outputImage.GetDimensions()
scalar_dim = outputImage.GetNumberOfScalarComponents()
assert(dims[2] == 1)

def get_ptr_from_str(s):
    return int(s[1:-7], 16)

out_array_ptr = C.cast(get_ptr_from_str(outputImage.GetScalarPointer()),C.POINTER(C.c_double))
# beware that out_array does not own its memory
# any modifications of its values will affect the output of the filter
out_array = np.ctypeslib.as_array(out_array_ptr,shape=(dims[0],dims[1],scalar_dim))
# note: using the following:
# print(outputImage.GetScalarComponentAsDouble(j, i, 0, 0), out_array[i][j][0])
# I found out that axis x and y are indexed differently, but this does not
# seem to be a problem (z axis does not seems to be swapped).

# Compute the DTM (digital terrain modeling) from DSM (digital surface modeling)
# We expect the height to be in the first scalar component of the vtkImageData.
# Note that out_array[:,:,0] is not a copy, but this is ok because a copy is done
# inside the DTM module
DTM = estimator.fit_dtm(out_array[:,:,0])

# now we modify the first scalar component of the image with the computed DTM
out_array[:,:,0] = DTM[:,:]
