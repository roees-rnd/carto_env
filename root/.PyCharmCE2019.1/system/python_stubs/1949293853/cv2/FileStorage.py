# encoding: utf-8
# module cv2 calls itself cv2.cv2
# from /opt/ros/kinetic/lib/python2.7/dist-packages/cv2.so
# by generator 1.147
""" Python wrapper for OpenCV. """

# imports
import cv2.cv2 as  # /usr/local/lib/python3.5/dist-packages/cv2/cv2.cpython-35m-x86_64-linux-gnu.so
import cv2.detail as detail # <module 'cv2.detail'>
import cv2.ml as ml # <module 'cv2.ml'>
import os as os # /usr/lib/python3.5/os.py
import cv2.data as data # /usr/local/lib/python3.5/dist-packages/cv2/data/__init__.py
import cv2.cuda as cuda # <module 'cv2.cuda'>
import cv2.ogl as ogl # <module 'cv2.ogl'>
import cv2.samples as samples # <module 'cv2.samples'>
import cv2.Error as Error # <module 'cv2.Error'>
import cv2.ipp as ipp # <module 'cv2.ipp'>
import cv2.flann as flann # <module 'cv2.flann'>
import cv2.instr as instr # <module 'cv2.instr'>
import importlib as importlib # /usr/lib/python3.5/importlib/__init__.py
import cv2.ocl as ocl # <module 'cv2.ocl'>
import cv2.videoio_registry as videoio_registry # <module 'cv2.videoio_registry'>
import cv2.utils as utils # <module 'cv2.utils'>
import cv2.cv2 as cv2 # /usr/local/lib/python3.5/dist-packages/cv2/cv2.cpython-35m-x86_64-linux-gnu.so
import cv2.dnn as dnn # <module 'cv2.dnn'>
import cv2.fisheye as fisheye # <module 'cv2.fisheye'>
from cv2.cv2 import (createButton, createTrackbar, dnn_registerLayer, 
    dnn_unregisterLayer, redirectError, setMouseCallback)


from .object import object

class FileStorage(object):
    # no doc
    def getFirstTopLevelNode(self): # real signature unknown; restored from __doc__
        """
        getFirstTopLevelNode() -> retval
        .   @brief Returns the first element of the top-level mapping.
        .   @returns The first element of the top-level mapping.
        """
        pass

    def getFormat(self): # real signature unknown; restored from __doc__
        """
        getFormat() -> retval
        .   @brief Returns the current format.
        .   * @returns The current format, see FileStorage::Mode
        """
        pass

    def getNode(self, nodename): # real signature unknown; restored from __doc__
        """
        getNode(nodename) -> retval
        .   @overload
        """
        pass

    def isOpened(self): # real signature unknown; restored from __doc__
        """
        isOpened() -> retval
        .   @brief Checks whether the file is opened.
        .   
        .   @returns true if the object is associated with the current file and false otherwise. It is a
        .   good practice to call this method after you tried to open a file.
        """
        pass

    def open(self, filename, flags, encoding=None): # real signature unknown; restored from __doc__
        """
        open(filename, flags[, encoding]) -> retval
        .   @brief Opens a file.
        .   
        .   See description of parameters in FileStorage::FileStorage. The method calls FileStorage::release
        .   before opening the file.
        .   @param filename Name of the file to open or the text string to read the data from.
        .   Extension of the file (.xml, .yml/.yaml or .json) determines its format (XML, YAML or JSON
        .   respectively). Also you can append .gz to work with compressed files, for example myHugeMatrix.xml.gz. If both
        .   FileStorage::WRITE and FileStorage::MEMORY flags are specified, source is used just to specify
        .   the output file format (e.g. mydata.xml, .yml etc.). A file name can also contain parameters.
        .   You can use this format, "*?base64" (e.g. "file.json?base64" (case sensitive)), as an alternative to
        .   FileStorage::BASE64 flag.
        .   @param flags Mode of operation. One of FileStorage::Mode
        .   @param encoding Encoding of the file. Note that UTF-16 XML encoding is not supported currently and
        .   you should use 8-bit encoding instead of it.
        """
        pass

    def release(self): # real signature unknown; restored from __doc__
        """
        release() -> None
        .   @brief Closes the file and releases all the memory buffers.
        .   
        .   Call this method after all I/O operations with the storage are finished.
        """
        pass

    def releaseAndGetString(self): # real signature unknown; restored from __doc__
        """
        releaseAndGetString() -> retval
        .   @brief Closes the file and releases all the memory buffers.
        .   
        .   Call this method after all I/O operations with the storage are finished. If the storage was
        .   opened for writing data and FileStorage::WRITE was specified
        """
        pass

    def root(self, streamidx=None): # real signature unknown; restored from __doc__
        """
        root([, streamidx]) -> retval
        .   @brief Returns the top-level mapping
        .   @param streamidx Zero-based index of the stream. In most cases there is only one stream in the file.
        .   However, YAML supports multiple streams and so there can be several.
        .   @returns The top-level mapping.
        """
        pass

    def write(self, name, val): # real signature unknown; restored from __doc__
        """
        write(name, val) -> None
        .   * @brief Simplified writing API to use with bindings.
        .   * @param name Name of the written object
        .   * @param val Value of the written object
        """
        pass

    def writeComment(self, comment, append=None): # real signature unknown; restored from __doc__
        """
        writeComment(comment[, append]) -> None
        .   @brief Writes a comment.
        .   
        .   The function writes a comment into file storage. The comments are skipped when the storage is read.
        .   @param comment The written comment, single-line or multi-line
        .   @param append If true, the function tries to put the comment at the end of current line.
        .   Else if the comment is multi-line, or if it does not fit at the end of the current
        .   line, the comment starts a new line.
        """
        pass

    def __init__(self, *args, **kwargs): # real signature unknown
        pass

    @staticmethod # known case of __new__
    def __new__(*args, **kwargs): # real signature unknown
        """ Create and return a new object.  See help(type) for accurate signature. """
        pass

    def __repr__(self, *args, **kwargs): # real signature unknown
        """ Return repr(self). """
        pass


