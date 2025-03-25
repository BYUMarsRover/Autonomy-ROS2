import os
import sys
from csv import DictReader

class ScienceModuleFunctionList:

    def __init__(self):
        self.function_list = self.load_function_map('science_module_function_map.csv')

    def load_function_map(self, filename):
        with open(filename, 'r') as f:
            dict_reader = DictReader(f)
            return list(dict_reader)
        
    def get_all(self):
        return self.function_list
    
    def filter(self, args):
        """Filter the function list based on the provided arguments."""
        filtered_list = []
        for func in self.function_list:
            skip = False
            for k, v in args.items():
                if not v == func[k]:
                    skip = True
                    break
            if not skip:
                filtered_list.append(func)
        return filtered_list