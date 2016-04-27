import numpy, openravepy
import pylab as pl
from DiscreteEnvironment import DiscreteEnvironment

class Control(object):
    def __init__(self, omega_left, omega_right, duration):
        self.ul = omega_left
        self.ur = omega_right
        self.dt = duration

class Action(object):
    def __init__(self, control, footprint):
        self.control = control
        self.footprint = footprint

class SimpleEnvironment(object):
    
    def __init__(self, herb, resolution):
        self.herb = herb
        self.robot = herb.robot
        self.boundary_limits = [[-5., -5., -numpy.pi], [5., 5., numpy.pi]]
        lower_limits, upper_limits = self.boundary_limits
        self.discrete_env = DiscreteEnvironment(resolution, lower_limits, upper_limits)

        self.resolution = resolution
        self.ConstructActions()

    def GenerateFootprintFromControl(self, start_config, control, stepsize=0.01):

        # Extract the elements of the control
        ul = control.ul
        ur = control.ur
        dt = control.dt

        # Initialize the footprint
        config = start_config.copy()
        footprint = [numpy.array([0., 0., config[2]])]
        timecount = 0.0
        while timecount < dt:
            # Generate the velocities based on the forward model
            xdot = 0.5 * self.herb.wheel_radius * (ul + ur) * numpy.cos(config[2])
            ydot = 0.5 * self.herb.wheel_radius * (ul + ur) * numpy.sin(config[2])
            tdot = self.herb.wheel_radius * (ul - ur) / self.herb.wheel_distance
                
            # Feed forward the velocities
            if timecount + stepsize > dt:
                stepsize = dt - timecount
            config = config + stepsize*numpy.array([xdot, ydot, tdot])
            if config[2] > numpy.pi:
                config[2] -= 2.*numpy.pi
            if config[2] < -numpy.pi:
                config[2] += 2.*numpy.pi

            footprint_config = config.copy()
            footprint_config[:2] -= start_config[:2]
            footprint.append(footprint_config)

            timecount += stepsize
            
        # Add one more config that snaps the last point in the footprint to the center of the cell
        nid = self.discrete_env.ConfigurationToNodeId(config)
        snapped_config = self.discrete_env.NodeIdToConfiguration(nid)
        snapped_config[:2] -= start_config[:2]
        footprint.append(snapped_config)

        return footprint

    def PlotActionFootprints(self, idx):

        actions = self.actions[idx]
        fig = pl.figure()
        lower_limits, upper_limits = self.boundary_limits
        pl.xlim([lower_limits[0], upper_limits[0]])
        pl.ylim([lower_limits[1], upper_limits[1]])
        
        for action in actions:
            xpoints = [config[0] for config in action.footprint]
            ypoints = [config[1] for config in action.footprint]
            pl.plot(xpoints, ypoints, 'k')
                     
        pl.ion()
        pl.show()

        

    def ConstructActions(self):

        # Actions is a dictionary that maps orientation of the robot to
        #  an action set
        self.actions = dict()
        self.controlList = []
        self.controlTest = Control(0,0,0)
        wc = [0., 0., 0.]
        grid_coordinate = self.discrete_env.ConfigurationToGridCoord(wc)

        # Set control list
        omega_list_l = numpy.arange(0,1.5,0.5) #0 0.5 1.0
        omega_list_r = numpy.arange(0,1.5,0.5) #0 0.5 1.0
        duration_list = numpy.arange(2,3,1) #2
        rotate_diretion = [-1, 1]
        
        """
        for omega_l in omega_list:
            for omega_r in omega_list:
                for dt in duration_list:
                    #print str(omega_l) + " " + str(omega_r) + " " + str(dt)
                    self.controlList.append(Control(omega_l,omega_r,dt))
        """
        for omega_l in omega_list_l:
            for omega_r in omega_list_r:
                for dt in duration_list:
                    for direction in rotate_diretion:
                        self.controlList.append(Control(omega_l*direction,omega_r*direction,dt))


        # Iterate through each possible starting orientation
        for idx in range(int(self.discrete_env.num_cells[2])):
            self.actions[idx] = []
            grid_coordinate[2] = idx
            start_config = self.discrete_env.GridCoordToConfiguration(grid_coordinate)

            # TODO: Here you will construct a set of actions
            #  to be used during the planning process
            #
            for control in self.controlList:
                footprint = self.GenerateFootprintFromControl(start_config, control)
                self.actions[idx].append(Action(control,footprint))



         
            

    def GetSuccessors(self, node_id):

        successors = []

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids and controls that represent the neighboring
        #  nodes
        coord = self.discrete_env.NodeIdToGridCoord(node_id)
        if not self.BoundaryCheck(coord) or self.CollisionCheck(coord):
            return successors

        successors = self.NeighborCheck(coord)

        return successors

    def BoundaryCheck(self, coord):
        config = self.discrete_env.GridCoordToConfiguration(coord)
        if config[0] < self.lower_limits[0] or config[1] < self.lower_limits[0]:
            return False
        if config[0] > self.upper_limits[0] or config[1] > self.upper_limits[1]:
            return False
        return True

    def NeighborCheck(self, coord):
        neighbors = []
        neighbors.append([coord[0]-1, coord[1]])
        neighbors.append([coord[0]+1, coord[1]])
        neighbors.append([coord[0], coord[1]-1])
        neighbors.append([coord[0], coord[1]+1])

        successors = []

        for i in xrange(len(neighbors)):
            if not self.CollisionCheck(neighbors[i]) and self.BoundaryCheck(neighbors[i]):
                successors.append(self.discrete_env.GridCoordToNodeId(neighbors[i]))
        return successors 



    def ComputeDistance(self, start_id, end_id):

        dist = 0

        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids
        dist = numpy.linalg.norm(start_config-end_config) 
        return dist

    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids
        cost = self.ComputeDistance(start_id, goal_id)
        
        return cost

