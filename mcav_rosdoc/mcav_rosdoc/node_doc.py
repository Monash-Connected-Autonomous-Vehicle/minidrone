''' TODO '''

import re
from collections import OrderedDict
from rclpy.node import Node, ParameterDescriptor

SECTION_FORMAT = ''.join([
    r'(?P<title>[A-Z][a-z]*)',      # Group containing section title
    r'\s*\n\s*-+\s*\n\s*',          # Title dash (---) deliniation with leading/trailing whitespace
    r'(?P<content>(?:.|\n)*?)\s*',  # Group containing all section content
    r'(?=[A-Z][a-z]*\s*\n\s*-|$)'   # Check that match precedes next section title, or end of string
    ])

PARAM_FORMAT = ''.join([
    r'(?P<name>[A-Za-z0-9-_]*)',         # Group containing param name
    r'\s*:\s*',                          #
    r'(?:[A-Za-z0-9]*)'                  # Group containing param datatype (unrecorded)
    r'\s*\n\s*'                          #
    r'(?P<description>(?:.|\n)*?)'       # Group containing param description
    r'\s*(?=(?:.[A-Za-z0-9-_]*\s*:)|$)'  # Check that match precedes next param, or end of string
    ])

TOPIC_FORMAT = ''.join([
    r'(?P<name>[A-Za-z0-9-_]*)',           # Group containing topic name
    r'\s*:\s*'                             #
    r'L\{',                                #
        r'(?P<package>[A-Za-z0-9-_]+)'     # Group containing message package                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                                   
        r'.'                               #
        r'(?P<dtype>[A-Za-z0-9-_]+)'       # Group containing message type
    r'\}',                                 #
    r'\s*\n\s*',                           #
    r'(?P<description>(?:.|\n)*?)'         # Group containing topic description
    r'\s*(?=(?:.[A-Za-z0-9-_]*\s*:)|$)'])  # Check that match precedes next topic, or end of string


def mcav_node_doc(node: Node):
    '''TODO'''

    # Store base init method for node
    raw_init = node.__init__

    def new_init(self: Node, *args, **kwargs):
        raw_init(self, *args, **kwargs)

        # Separate docstring into sections
        self.sections = dict(re.findall(SECTION_FORMAT, self.__doc__))

        # Compile lists of subscribed/published topics
        topic_info = OrderedDict()
        for topic in re.finditer(TOPIC_FORMAT, self.sections['Subscribes']):
            topic_dict = topic.groupdict()
            topic_info[topic['name']] = {field:topic_dict[field] for field in topic_dict if field != 'name'}
            topic_info[topic['name']]['subscribed'] = True


        # Iterate through parameter descriptions found by regex match
        for match in re.finditer(PARAM_FORMAT, self.sections['Parameters']):
            # Populate descriptor
            desc = ParameterDescriptor()
            name, desc.description = match.groups()
            desc.name, desc.type = name, self.get_parameter(name).type_.value
            # Apply descriptor
            self.set_descriptor(name, desc)

        for match in re.finditer(TOPIC_FORMAT, self.sections['Subscribes']):
            print(match.groupdict())

    # Update node init method and output
    node.__init__ = new_init
    return node
