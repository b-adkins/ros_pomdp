#!/usr/bin/python 

##
# @file
#
# Classes that describes a Feature space - state, action, observation, etc.
#
# Port of the C++ class family.
#
# @author Bea Adkins
# @date 2013-09-19

from abc import *
import operator

## Helper function equivalent to Python's sum() for multiplication.
def _product(iterable):
    return reduce(operator.mul, iterable, 1)

class FeatureSpaceFactory:
    ##
    # Initialize a feature space from parameter server configuration.
    #
    # @param param List whose items specify feature spaces.
    # @throws ValueError When param doesn't specify a valid feature space.
    def getFeatureSpace(self, param):
        # Vector of feature spaces
        if isinstance(param, (list, tuple)):
            return VectorFeatureSpace(param)
        elif isinstance(param['values'], (list, tuple)):
            # List of strings
            if all(isinstance(val, basestring) for val in param['values']):
                return SimpleFeatureSpace(param)
            else:
                raise ValueError('Tree feature spaces not yet supported.')            
        else:
            return SimpleFeatureSpace(param)


class FeatureSpace:
    __metaclass__ = ABCMeta
    
    @abstractmethod
    def __str__(self):
        pass
    
    ##
    # Checks if value is member of this feature space.
    #
    # @param value Name or number.
    # @return True if value is valid for this featurespace.
    @abstractmethod
    def is_valid(self, value):
        pass

    ##
    # Number of values a feature within this feature space can take.
    #
    # @return Integer number of values or 0 on failure.
    @abstractmethod
    def num_values(self):
        pass

    ##
    # Pretty prints a feature value.
    #
    # @param value Feature value that is a member of this feature space.
    # @return String representation of the value.
    @abstractmethod
    def toString(self, value):
        pass

    ## 
    # Reads a feature value from a string.
    #
    # @param string Specially formatted string.
    # @return Feature value
    # @throws ValueError If string can't be parsed.
    @abstractmethod
    def fromString(self, string):
        pass

    ##
    # Converts feature value into an integer ID.
    #
    # @param value Feature value.
    # @return Integer ID.
    @abstractmethod
    def hash(self, value):
        pass

    ##
    # Converts integer ID into feature value.
    #
    # @param id Integer ID.
    # @return Feature value.
    @abstractmethod
    def dehash(self, id):
        pass


##
# FeatureSpace containing discrete states mapped to integers.
#
class SimpleFeatureSpace(FeatureSpace):
    ##
    # Initialized from a parameter server FeatureSpace.
    #
    # @param param Dictionary containing string 'name' and string list or int 'values'
    # @throws ValueError When param doesn't specify a valid FeatureSpace.
    def __init__(self, param):
        self.name = param['name']
        values = param['values']

        # Number of possible values, instead of list of names.
        if isinstance(values, int) and values > 0:
            self.values = range(0, values)
        # List of names
        elif isinstance(values, (list, tuple)) and all(isinstance(val, basestring) for val in values):
            self.values = values
        elif isinstance(values, dict):
            raise ValueError("Dictionary name:number mappings not supported. (They're overkill.)")
        else:
            raise ValueError
        
        # Fail if too short.
        if len(self.values) < 2:
            raise ValueError
    
    def __str__(self):
        def isInt(a):
            try:
                int(a)
                return True
            except ValueError:
                return False
        # String form of each name-number mapping.
        mapping_strings = ["%i" % (i_s) if isInt(s) else "%i:'%s'" % (i_s, s) for i_s, s in enumerate(self.values)]
        return "%s: " % self.name + ", ".join(mapping_strings)            
    
    ##
    # Converts a feature number to a name.
    #
    # @throws IndexError
    def to_name(self, number):
        return self.values[number] 

    ##
    # Converts a feature name to a number.
    #
    # @param name Feature value, string form.
    # @throws ValueError
    def to_number(self, name):
        # In case name is an int or a numeric string
        try:
            return int(name)
        except ValueError:
            # Try to look up number.
            return self.values.index(name) 

    ##
    # Check if a value is valid for this feature space.
    #
    # @param value Name or number.
    # @return True if value is a member of this feature space. False if type is incorrect or value is out of bounds.
    def is_valid(self, value):
        # For strings, look up number.
        if(isinstance(value, (str, unicode))):
            try:
                number = self.to_number(value)
            except ValueError:
                return False
        # For numbers, just use them.
        elif(isinstance(value, int)):
            number = value
        else:
            return False
        
        # Verify within range.
        return 0 <= number and number < len(self.values)

    def num_values(self):
        return len(self.values)

    def toString(self, value):
        return self.to_name(value)

    ##
    # @param string String matching name or integer belonging to this feature space.
    def fromString(self, string):
        return self.to_number(string)

    def hash(self, value):
        # For a SimpleStateSpace, value and ID are the same.
        return value

    def dehash(self, id):
        # For a SimpleStateSpace, value and ID are the same.
        return id


##
# Class that describes a Vector feature space - a feature space composed of other feature spaces.
class VectorFeatureSpace(FeatureSpace):
    def __init__(self, param):
        factory = FeatureSpaceFactory()
        self.name = ''
        self.values = [factory.getFeatureSpace(val) for val in param]
    
        self._generatePlaceValues()
    
    def __str__(self):
        ret = ""
        for fs in self.values:
            ret += "- %s\n" % fs.__str__()
        return ret

    def __repr__(self):
        return self.__str__()
    

    def is_valid(self, value):
        # Vector value.
        if isinstance(value, (list, tuple)):
            # Checks if lengths match.
            if len(self.values) != len(value):
                return False
            # Checks if children are valid.
            else:
                return all(fs.is_valid(v) for fs, v in zip(self.values, value))
        # Hashed ID.
        elif isinstance(value, int):
            return 0 <= value and value < self.num_values()
        
        # Not a vector value.
        else:
            return False

    def toString(self, value):
        # Hashed ID.
        if isinstance(value, int):
            value = self.dehash(value)

        ret = "[ "
        for val, fs in zip(value, self.values):
            ret += fs.toString(val) + ", "
        ret += "]"
        return ret
    
    ##
    # @param string String containing list of values belonging to member state spaces, comma seperated (whitespace OK).
    # Does not support recursive entry, yet.
    def fromString(self, string):
        str_items = string.split(",")
        str_items = [s.strip() for s in str_items]
        return [fs.fromString(item) for item, fs in zip(str_items, values)]

    def num_values(self):
        # Calculate place values. @todo Replace with lazy initialization or something else faster. 
        children_num_values = [fs.num_values() for fs in self.values]
                 
        # Multiply to get number of values.
        return _product(children_num_values)

    def hash(self, value):
        # Dot product
        return sum(v*pv for v,pv in zip(value, self._place_values))

    def dehash(self, id):
        value = [None] * len(self.values) # Initialize to number of sub-FeatureSpaces
        remainder = id
        
        # Countdown loop
        for f in reversed(range(0, len(self.values))):
            value[f] = remainder / self._place_values[f]; # Int division
            remainder = remainder % self._place_values[f];
            
        return value

    ## Initializes initial place value vector.
    def _generatePlaceValues(self):
         children_num_values = [fs.num_values() for fs in self.values]
         self._place_values = [1 if f == 0 else _product(children_num_values[0:f]) for f in range(0, len(children_num_values))]
