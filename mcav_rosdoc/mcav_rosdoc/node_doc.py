''' TODO '''

import re
from rclpy.node import Node, ParameterDescriptor


NODE_SECTION_NAMES = ('Parameters', 'Subscribes', 'Pubishes')
# TODO: Make regex legible and document
NODE_SECTION_FORMAT = r'([A-Z][a-z]*)\s*\n\s*-+\s*\n\s*((?:.|\n)*?)\s*(?=[A-Z][a-z]*\s*\n\s*-|$)'
NODE_PARAM_FORMAT = r'([A-Za-z0-9-_]*)\s:\s(?:[A-Za-z0-9]*)\s*\n\s*((?:.|\n)*?)\s*(?=(?:.[A-Za-z0-9-_]*\s:)|$)'


def mcav_node_doc(node: Node):
    '''TODO'''

    # Store base init method for node
    raw_init = node.__init__

    def new_init(self: Node, *args, **kwargs):
        raw_init(self, *args, **kwargs)

        # Separate docstring into sections
        sections = dict(re.findall(NODE_SECTION_FORMAT, self.__doc__))

        # Iterate through parameter descriptions found by regex match
        for match in re.finditer(NODE_PARAM_FORMAT, sections['Parameters']):
            # Populate descriptor
            desc = ParameterDescriptor()
            name, desc.description = match.groups()
            desc.name, desc.type = name, self.get_parameter(name).type_.value
            # Apply descriptor
            self.set_descriptor(name, desc)

    # Update node init method and output
    node.__init__ = new_init
    return node
