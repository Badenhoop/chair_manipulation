# Copied FROM: https://github.com/enthought/mayavi/blob/dd1332e9ab4a7821eeb1e53a5d5047390e6f4877/tvtk/array_handler.py
import numpy
import vtk
from vtk.util import vtkConstants
from vtk.util import numpy_support

# Useful constants for VTK arrays.
VTK_ID_TYPE_SIZE = vtk.vtkIdTypeArray().GetDataTypeSize()
if VTK_ID_TYPE_SIZE == 4:
    ID_TYPE_CODE = numpy.int32
elif VTK_ID_TYPE_SIZE == 8:
    ID_TYPE_CODE = numpy.int64

VTK_LONG_TYPE_SIZE = vtk.vtkLongArray().GetDataTypeSize()
if VTK_LONG_TYPE_SIZE == 4:
    LONG_TYPE_CODE = numpy.int32
    ULONG_TYPE_CODE = numpy.uint32
elif VTK_LONG_TYPE_SIZE == 8:
    LONG_TYPE_CODE = numpy.int64
    ULONG_TYPE_CODE = numpy.uint64


def get_vtk_to_numeric_typemap():
    """Returns the VTK array type to numpy array type mapping."""
    _vtk_arr = {
        vtkConstants.VTK_BIT: numpy.bool,
        vtkConstants.VTK_CHAR: numpy.int8,
        vtkConstants.VTK_UNSIGNED_CHAR: numpy.uint8,
        vtkConstants.VTK_SHORT: numpy.int16,
        vtkConstants.VTK_UNSIGNED_SHORT: numpy.uint16,
        vtkConstants.VTK_INT: numpy.int32,
        vtkConstants.VTK_UNSIGNED_INT: numpy.uint32,
        vtkConstants.VTK_LONG: LONG_TYPE_CODE,
        vtkConstants.VTK_UNSIGNED_LONG: ULONG_TYPE_CODE,
        vtkConstants.VTK_ID_TYPE: ID_TYPE_CODE,
        vtkConstants.VTK_FLOAT: numpy.float32,
        vtkConstants.VTK_DOUBLE: numpy.float64
    }
    return _vtk_arr


def get_numeric_array_type(vtk_array_type):
    """Returns a numpy array typecode given a VTK array type."""
    return get_vtk_to_numeric_typemap()[vtk_array_type]


def vtk2array(vtk_array):
    """Converts a VTK data array to a numpy array.
    Given a subclass of vtkDataArray, this function returns an
    appropriate numpy array containing the same data.  The function
    is very efficient since it uses the VTK imaging pipeline to
    convert the data.  If a sufficiently new version of VTK (5.2) is
    installed then it actually uses the buffer interface to return a
    view of the VTK array in the returned numpy array.
    Parameters
    ----------
    - vtk_array : `vtkDataArray`
      The VTK data array to be converted.
    """

    typ = vtk_array.GetDataType()
    assert typ in get_vtk_to_numeric_typemap().keys(), \
        "Unsupported array type %s" % typ

    shape = (vtk_array.GetNumberOfTuples(),
             vtk_array.GetNumberOfComponents())
    if shape[0] == 0:
        dtype = get_numeric_array_type(typ)
        return numpy.array([], dtype)

    # If VTK's new numpy support is available, use the buffer interface.
    if numpy_support is not None and typ != vtkConstants.VTK_BIT:
        dtype = get_numeric_array_type(typ)
        result = numpy.frombuffer(vtk_array, dtype=dtype)
        if shape[1] == 1:
            shape = (shape[0], )
        result.shape = shape
        return result
