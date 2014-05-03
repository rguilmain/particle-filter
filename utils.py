# Georgia Tech, CS-8802: Artificial Intelligence for Robotics, Final Project
# Authors: Richard Guilmain and Nabin Sharma

"""Utilities
"""

# Convert a dictionary to a class
# Reference: http://kiennt.com/blog/2012/06/14/python-object-and-dictionary-convertion.html
class Struct(object):
    def __init__(self, adict):
        """Convert a dictionary to a class. Can handle dict of dicts that
        might have list of dicts.
        """
        self.__dict__.update(adict)
        for k, v in adict.items():
            if isinstance(v, dict):
                self.__dict__[k] = Struct(v)
            if isinstance(v, list):
                self.__dict__[k] = []
                for e in v:
                    self.__dict__[k].append(Struct(e))
