import numpy

class DiscreteEnvironment(object):

    def __init__(self, resolution, lower_limits, upper_limits):

        # Store the resolution
        self.resolution = resolution

        # Store the bounds
        self.lower_limits = lower_limits
        self.upper_limits = upper_limits

        # Calculate the dimension
        self.dimension = len(self.lower_limits)

        # Figure out the number of grid cells that are in each dimension
        self.num_cells = self.dimension*[0]
        for i in range(self.dimension):
            self.num_cells[i] = numpy.ceil((upper_limits[i] - lower_limits[i])/resolution[i])


    def ConfigurationToNodeId(self, config):
        """This function maps a node configuration in full configuration space to a node in discrete space"""
        coord = self.ConfigurationToGridCoord(config)
        node_id = self.GridCoordToNodeId(coord)
        return node_id

    def NodeIdToConfiguration(self, nid):
        """This function maps a node in discrete space to a configuration in the full configuration space"""
        coord = self.NodeIdToGridCoord(nid)
        config = self.GridCoordToConfiguration(coord)
        return config
        
    def ConfigurationToGridCoord(self, config):
        """This function maps a configuration in the full configuration space to a grid coordinate in discrete space"""
        coord = [0] * self.dimension
        for i in xrange(self.dimension):
            coord[i] = numpy.floor((config[i]-self.lower_limits[i])/self.resolution[i])
        return coord
    
    def GridCoordToConfiguration(self, coord):
        """This function maps a grid coordinate in discrete space to a configuration in the full configuration space"""
        config = [0] * self.dimension
        for i in xrange(self.dimension):
            config[i] =  self.resolution[i]*coord[i] + self.resolution[i]/2
        config = numpy.add(config,self.lower_limits)
        return config

    def GridCoordToNodeId(self,coord):
        """This function maps a grid coordinate to the associated node id"""
        node_id = 0
        for i in range(1, self.dimension):
            layer = 1.0
            for j in range(i):
                layer = layer * self.num_cells[j]
            node_id = node_id + (coord[i] * layer)
        node_id = node_id + coord[0]
        return int(node_id)

    def NodeIdToGridCoord(self, node_id):
        """This function maps a node id to the associated grid coordinate"""
        coord = [0] * self.dimension
        layer = 1
        for i in range(self.dimension-1):
            layer = layer * self.num_cells[i]

        for i in reversed(range(1,self.dimension)):
            coord[i] = numpy.floor(node_id / layer)
            node_id = node_id - (coord[i]*layer)
            layer = layer / self.num_cells[i-1]

        coord[0] = node_id
        return coord

    def Test(self):
        print "test1====================================="
        config = [5.0, 5.0]
        print "config = " + str(config)
        coord = self.ConfigurationToGridCoord(config)
        print "coord = " + str(coord)
        nid = self.GridCoordToNodeId(coord)
        print "nid = " + str(nid)
        coord = self.NodeIdToGridCoord(nid)
        print "coord = " + str(coord)
        config = self.GridCoordToConfiguration(coord)
        print "config = " + str(config)


        print "test2====================================="
        config = [-5.0, -5.0]
        print "config = " + str(config)
        coord = self.ConfigurationToGridCoord(config)
        print "coord = " + str(coord)
        nid = self.GridCoordToNodeId(coord)
        print "nid = " + str(nid)
        coord = self.NodeIdToGridCoord(nid)
        print "coord = " + str(coord)
        config = self.GridCoordToConfiguration(coord)
        print "config = " + str(config)

        print "test3====================================="
        config = [0.26, 0.64]
        print "config = " + str(config)
        coord = self.ConfigurationToGridCoord(config)
        print "coord = " + str(coord)
        nid = self.GridCoordToNodeId(coord)
        print "nid = " + str(nid)
        coord = self.NodeIdToGridCoord(nid)
        print "coord = " + str(coord)
        config = self.GridCoordToConfiguration(coord)
        print "config = " + str(config)

        print "test4====================================="
        config = [-0.26, -0.64]
        print "config = " + str(config)
        coord = self.ConfigurationToGridCoord(config)
        print "coord = " + str(coord)
        nid = self.GridCoordToNodeId(coord)
        print "nid = " + str(nid)
        coord = self.NodeIdToGridCoord(nid)
        print "coord = " + str(coord)
        config = self.GridCoordToConfiguration(coord)
        print "config = " + str(config)

        print "test5====================================="
        config = [0.0, 0.0]
        print "config = " + str(config)
        coord = self.ConfigurationToGridCoord(config)
        print "coord = " + str(coord)
        nid = self.GridCoordToNodeId(coord)
        print "nid = " + str(nid)
        coord = self.NodeIdToGridCoord(nid)
        print "coord = " + str(coord)
        config = self.GridCoordToConfiguration(coord)
        print "config = " + str(config)