
def mcav_node_doc(node):
    '''TODO'''

    raw_init = node.__init__

    def new_init(self, *args, **kws):
        raw_init(self, *args, **kws)
        print('hey')
        # TODO: populate param documentation from __doc__

    node.__init__ = new_init
    return node
    