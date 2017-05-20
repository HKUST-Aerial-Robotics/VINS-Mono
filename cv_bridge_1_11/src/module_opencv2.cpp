/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of the Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

#include "module.hpp"

// These are sucky, sketchy versions of the real things in OpenCV Python,
// inferior in every way.

static int failmsg(const char *fmt, ...)
{
  char str[1000];

  va_list ap;
  va_start(ap, fmt);
  vsnprintf(str, sizeof(str), fmt, ap);
  va_end(ap);

  PyErr_SetString(PyExc_TypeError, str);
  return 0;
}


// Taken from http://stackoverflow.com/questions/19136944/call-c-opencv-functions-from-python-send-a-cv-mat-to-c-dll-which-is-usi


static size_t REFCOUNT_OFFSET = ( size_t )&((( PyObject* )0)->ob_refcnt ) +
( 0x12345678 != *( const size_t* )"\x78\x56\x34\x12\0\0\0\0\0" )*sizeof( int );


static inline PyObject* pyObjectFromRefcount( const int* refcount )
{
return ( PyObject* )(( size_t )refcount - REFCOUNT_OFFSET );
}

static inline int* refcountFromPyObject( const PyObject* obj )
{
return ( int* )(( size_t )obj + REFCOUNT_OFFSET );
}

class NumpyAllocator : public cv::MatAllocator
{
public:
NumpyAllocator( ) { }
~NumpyAllocator( ) { }

void allocate( int dims, const int* sizes, int type, int*& refcount,
uchar*& datastart, uchar*& data, size_t* step );

void deallocate( int* refcount, uchar* datastart, uchar* data );
};

void NumpyAllocator::allocate( int dims, const int* sizes, int type, int*& refcount, uchar*& datastart, uchar*& data, size_t* step )
{
    int depth = CV_MAT_DEPTH( type );
    int cn = CV_MAT_CN( type );
    const int f = ( int )( sizeof( size_t )/8 );
    int typenum = depth == CV_8U ? NPY_UBYTE : depth == CV_8S ? NPY_BYTE :
                  depth == CV_16U ? NPY_USHORT : depth == CV_16S ? NPY_SHORT :
                  depth == CV_32S ? NPY_INT : depth == CV_32F ? NPY_FLOAT :
                  depth == CV_64F ? NPY_DOUBLE : f*NPY_ULONGLONG + (f^1)*NPY_UINT;
    int i;
    npy_intp _sizes[CV_MAX_DIM+1];
    for( i = 0; i < dims; i++ )
        _sizes[i] = sizes[i];
    if( cn > 1 )
    {
    /*if( _sizes[dims-1] == 1 )
         _sizes[dims-1] = cn;
    else*/
        _sizes[dims++] = cn;
    }
    PyObject* o = PyArray_SimpleNew( dims, _sizes, typenum );
    if( !o )
    CV_Error_(CV_StsError, ("The numpy array of typenum=%d, ndims=%d can not be created", typenum, dims));
    refcount = refcountFromPyObject(o);
    npy_intp* _strides = PyArray_STRIDES((PyArrayObject*) o);
    for( i = 0; i < dims - (cn > 1); i++ )
        step[i] = (size_t)_strides[i];
    datastart = data = (uchar*)PyArray_DATA((PyArrayObject*)o);

}

void NumpyAllocator::deallocate( int* refcount, uchar* datastart, uchar* data )
{
    if( !refcount )
       return;
    PyObject* o = pyObjectFromRefcount(refcount);
    Py_INCREF(o);
    Py_DECREF(o);
}

// Declare the object
NumpyAllocator g_numpyAllocator;

int convert_to_CvMat2(const PyObject* o, cv::Mat& m)
{
    // to avoid PyArray_Check() to crash even with valid array
    do_numpy_import();

    if(!o || o == Py_None)
    {
        if( !m.data )
            m.allocator = &g_numpyAllocator;
        return true;
    }

    if( !PyArray_Check(o) )
    {
        failmsg("Not a numpy array");
        return false;
    }

    // NPY_LONG (64 bit) is converted to CV_32S (32 bit)
    int typenum = PyArray_TYPE((PyArrayObject*) o);
    int type = typenum == NPY_UBYTE ? CV_8U : typenum == NPY_BYTE ? CV_8S :
        typenum == NPY_USHORT ? CV_16U : typenum == NPY_SHORT ? CV_16S :
        typenum == NPY_INT || typenum == NPY_LONG ? CV_32S :
        typenum == NPY_FLOAT ? CV_32F :
        typenum == NPY_DOUBLE ? CV_64F : -1;

    if( type < 0 )
    {
        failmsg("data type = %d is not supported", typenum);
        return false;
    }

    int ndims = PyArray_NDIM((PyArrayObject*) o);
    if(ndims >= CV_MAX_DIM)
    {
        failmsg("dimensionality (=%d) is too high", ndims);
        return false;
    }

    int size[CV_MAX_DIM+1];
    size_t step[CV_MAX_DIM+1], elemsize = CV_ELEM_SIZE1(type);
    const npy_intp* _sizes = PyArray_DIMS((PyArrayObject*) o);
    const npy_intp* _strides = PyArray_STRIDES((PyArrayObject*) o);
    bool transposed = false;

    for(int i = 0; i < ndims; i++)
    {
        size[i] = (int)_sizes[i];
        step[i] = (size_t)_strides[i];
    }

    if( ndims == 0 || step[ndims-1] > elemsize ) {
        size[ndims] = 1;
        step[ndims] = elemsize;
        ndims++;
    }

    if( ndims >= 2 && step[0] < step[1] )
    {
        std::swap(size[0], size[1]);
        std::swap(step[0], step[1]);
        transposed = true;
    }

    if( ndims == 3 && size[2] <= CV_CN_MAX && step[1] == elemsize*size[2] )
    {
        ndims--;
        type |= CV_MAKETYPE(0, size[2]);
    }

    if( ndims > 2 )
    {
        failmsg("more than 2 dimensions");
        return false;
    }

    m = cv::Mat(ndims, size, type, PyArray_DATA((PyArrayObject*) o), step);

    if( m.data )
    {
        m.refcount = refcountFromPyObject(o);
        m.addref(); // protect the original numpy array from deallocation
        // (since Mat destructor will decrement the reference counter)
    };
    m.allocator = &g_numpyAllocator;

    if( transposed )
    {
        cv::Mat tmp;
        tmp.allocator = &g_numpyAllocator;
        transpose(m, tmp);
        m = tmp;
    }
    return true;
}

PyObject* pyopencv_from(const cv::Mat& mat)
{
    npy_intp dims[] = {mat.rows, mat.cols, mat.channels()};

    PyObject *res = 0 ;
    if (mat.depth() == CV_8U)
        res = PyArray_SimpleNew(3, dims, NPY_UBYTE);
    else if (mat.depth() == CV_8S)
        res = PyArray_SimpleNew(3, dims, NPY_BYTE);
    else if (mat.depth() == CV_16S)
        res = PyArray_SimpleNew(3, dims, NPY_SHORT);
    else if (mat.depth() == CV_16U)
        res = PyArray_SimpleNew(3, dims, NPY_USHORT);
    else if (mat.depth() == CV_32S)
        res = PyArray_SimpleNew(3, dims, NPY_INT);
    else if (mat.depth() == CV_32F)
        res = PyArray_SimpleNew(3, dims, NPY_CFLOAT);
    else if (mat.depth() == CV_64F)
        res = PyArray_SimpleNew(3, dims, NPY_CDOUBLE);

    std::memcpy(PyArray_DATA((PyArrayObject*)res), mat.data, mat.step*mat.rows);

    return res;
}
