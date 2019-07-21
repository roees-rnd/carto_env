# encoding: utf-8
# module _csv
# from /usr/lib/python3.5/lib-dynload/_csv.cpython-35m-x86_64-linux-gnu.so
# by generator 1.147
"""
CSV parsing and writing.

This module provides classes that assist in the reading and writing
of Comma Separated Value (CSV) files, and implements the interface
described by PEP 305.  Although many CSV files are simple to parse,
the format is not formally defined by a stable specification and
is subtle enough that parsing lines of a CSV file with something
like line.split(",") is bound to fail.  The module supports three
basic APIs: reading, writing, and registration of dialects.


DIALECT REGISTRATION:

Readers and writers support a dialect argument, which is a convenient
handle on a group of settings.  When the dialect argument is a string,
it identifies one of the dialects previously registered with the module.
If it is a class or instance, the attributes of the argument are used as
the settings for the reader or writer:

    class excel:
        delimiter = ','
        quotechar = '"'
        escapechar = None
        doublequote = True
        skipinitialspace = False
        lineterminator = '\r\n'
        quoting = QUOTE_MINIMAL

SETTINGS:

    * quotechar - specifies a one-character string to use as the 
        quoting character.  It defaults to '"'.
    * delimiter - specifies a one-character string to use as the 
        field separator.  It defaults to ','.
    * skipinitialspace - specifies how to interpret whitespace which
        immediately follows a delimiter.  It defaults to False, which
        means that whitespace immediately following a delimiter is part
        of the following field.
    * lineterminator -  specifies the character sequence which should 
        terminate rows.
    * quoting - controls when quotes should be generated by the writer.
        It can take on any of the following module constants:

        csv.QUOTE_MINIMAL means only when required, for example, when a
            field contains either the quotechar or the delimiter
        csv.QUOTE_ALL means that quotes are always placed around fields.
        csv.QUOTE_NONNUMERIC means that quotes are always placed around
            fields which do not parse as integers or floating point
            numbers.
        csv.QUOTE_NONE means that quotes are never placed around fields.
    * escapechar - specifies a one-character string used to escape 
        the delimiter when quoting is set to QUOTE_NONE.
    * doublequote - controls the handling of quotes inside fields.  When
        True, two consecutive quotes are interpreted as one during read,
        and when writing, each quote character embedded in the data is
        written as two quotes
"""
# no imports

# Variables with simple values

QUOTE_ALL = 1
QUOTE_MINIMAL = 0
QUOTE_NONE = 3
QUOTE_NONNUMERIC = 2

__version__ = '1.0'

# functions

def field_size_limit(limit=None): # real signature unknown; restored from __doc__
    """
    Sets an upper limit on parsed fields.
        csv.field_size_limit([limit])
    
    Returns old limit. If limit is not given, no new limit is set and
    the old limit is returned
    """
    pass

def get_dialect(name): # real signature unknown; restored from __doc__
    """
    Return the dialect instance associated with name.
        dialect = csv.get_dialect(name)
    """
    pass

def list_dialects(): # real signature unknown; restored from __doc__
    """
    Return a list of all know dialect names.
        names = csv.list_dialects()
    """
    pass

def reader(iterable, dialect='excel', *args, **kwargs): # real signature unknown; NOTE: unreliably restored from __doc__ 
    """
    csv_reader = reader(iterable [, dialect='excel']
                            [optional keyword args])
        for row in csv_reader:
            process(row)
    
    The "iterable" argument can be any object that returns a line
    of input for each iteration, such as a file object or a list.  The
    optional "dialect" parameter is discussed below.  The function
    also accepts optional keyword arguments which override settings
    provided by the dialect.
    
    The returned object is an iterator.  Each iteration returns a row
    of the CSV file (which can span multiple input lines).
    """
    pass

def register_dialect(name, dialect=None, **fmtparams=None): # real signature unknown; restored from __doc__
    """
    Create a mapping from a string name to a dialect class.
        dialect = csv.register_dialect(name[, dialect[, **fmtparams]])
    """
    pass

def unregister_dialect(name): # real signature unknown; restored from __doc__
    """
    Delete the name/dialect mapping associated with a string name.
        csv.unregister_dialect(name)
    """
    pass

def writer(fileobj, dialect='excel', *args, **kwargs): # real signature unknown; NOTE: unreliably restored from __doc__ 
    """
    csv_writer = csv.writer(fileobj [, dialect='excel']
                                [optional keyword args])
        for row in sequence:
            csv_writer.writerow(row)
    
        [or]
    
        csv_writer = csv.writer(fileobj [, dialect='excel']
                                [optional keyword args])
        csv_writer.writerows(rows)
    
    The "fileobj" argument can be any object that supports the file API.
    """
    pass

# classes

class Dialect(object):
    """
    CSV dialect
    
    The Dialect type records CSV parsing and generation options.
    """
    def __init__(self, *args, **kwargs): # real signature unknown
        pass

    @staticmethod # known case of __new__
    def __new__(*args, **kwargs): # real signature unknown
        """ Create and return a new object.  See help(type) for accurate signature. """
        pass

    delimiter = property(lambda self: object(), lambda self, v: None, lambda self: None)  # default

    doublequote = property(lambda self: object(), lambda self, v: None, lambda self: None)  # default

    escapechar = property(lambda self: object(), lambda self, v: None, lambda self: None)  # default

    lineterminator = property(lambda self: object(), lambda self, v: None, lambda self: None)  # default

    quotechar = property(lambda self: object(), lambda self, v: None, lambda self: None)  # default

    quoting = property(lambda self: object(), lambda self, v: None, lambda self: None)  # default

    skipinitialspace = property(lambda self: object(), lambda self, v: None, lambda self: None)  # default

    strict = property(lambda self: object(), lambda self, v: None, lambda self: None)  # default



class Error(Exception):
    # no doc
    def __init__(self, *args, **kwargs): # real signature unknown
        pass

    __weakref__ = property(lambda self: object(), lambda self, v: None, lambda self: None)  # default
    """list of weak references to the object (if defined)"""



# variables with complex values

_dialects = {}

__loader__ = None # (!) real value is '<_frozen_importlib_external.ExtensionFileLoader object at 0x7f8c2b7e6b70>'

__spec__ = None # (!) real value is "ModuleSpec(name='_csv', loader=<_frozen_importlib_external.ExtensionFileLoader object at 0x7f8c2b7e6b70>, origin='/usr/lib/python3.5/lib-dynload/_csv.cpython-35m-x86_64-linux-gnu.so')"

