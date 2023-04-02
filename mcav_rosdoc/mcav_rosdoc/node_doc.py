''' TODO '''

import re
from collections import OrderedDict
from rclpy.node import Node, ParameterDescriptor

# TODO: Make regex legible and document
SECTION_FORMAT = r'([A-Z][a-z]*)\s*\n\s*-+\s*\n\s*((?:.|\n)*?)\s*(?=[A-Z][a-z]*\s*\n\s*-|$)'
PARAM_FORMAT = r'(?P<name>[A-Za-z0-9-_]*)\s*:\s*(?:[A-Za-z0-9]*)\s*\n\s*(?P<description>(?:.|\n)*?)\s*(?=(?:.[A-Za-z0-9-_]*\s*:)|$)'
TOPIC_FORMAT = r'(?P<name>[A-Za-z0-9-_]*)\s*:\s*L\{(?P<package>[A-Za-z0-9-_]+).(?P<dtype>[A-Za-z0-9-_]+)\}\s*\n\s*(?P<description>(?:.|\n)*?)\s*(?=(?:.[A-Za-z0-9-_]*\s*:)|$)'


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
            topic_info[topic['name']] = {field:topic[field] for field in topic if field != 'name'}
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
