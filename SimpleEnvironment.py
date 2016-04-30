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
        self.lower_limits, self.upper_limits = self.boundary_limits
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

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
        omega_list_l = numpy.arange(-1,1.5,0.25) #0 0.5 1.0
        omega_list_r = numpy.arange(-1,1.5,0.25) #0 0.5 1.0
        duration_list = numpy.arange(0,0.5,0.01) #2
        
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
                    self.controlList.append(Control(omega_l,omega_r,dt))


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

        self.successors = {}

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids and controls that represent the neighboring
        #  nodes
        curr_coord = self.discrete_env.NodeIdToGridCoord(node_id)
        curr_config = self.discrete_env.NodeIdToConfiguration(node_id)
        if not self.BoundaryCheck(curr_config) or self.CollisionCheck(curr_config):
            return self.successors
        #need to modify
        omega_index = numpy.round(((curr_config[2] - -numpy.pi)/(2*numpy.pi))*self.discrete_env.num_cells[2])
        if omega_index == self.discrete_env.num_cells[2]:
            omega_index = omega_index - 1
        curr_config[2] = (self.discrete_env.resolution[2] * omega_index) - numpy.pi
        #print "node_id = " + str(node_id)
        print "curr_config = " + str(curr_config)
        print "omega_index = " + str(omega_index)
        for action in self.actions[omega_index]:
            test_config = curr_config + action.footprint[-1] #test final point
            if not self.CollisionCheck(test_config) and self.BoundaryCheck(test_config):
                index = self.discrete_env.ConfigurationToNodeId(test_config)
                #print "index = " + str(index)
                self.successors[index] = action

        return self.successors

    def CollisionCheck(self,config):
        #config = self.discrete_env.GridCoordToConfiguration(coord)

        current_pose = self.robot.GetTransform()
        checking_pose = current_pose.copy()
        env = self.robot.GetEnv()

        checking_pose[0:3,3] = numpy.array([config[0], config[1], 0.0])
        angle = config[2]
        rotation = numpy.array([[numpy.cos(angle), -numpy.sin(angle),0.0],[numpy.sin(angle),numpy.cos(angle),0.0],[0.0,0.0,1.0]])
        checking_pose[0:3,0:3] = rotation

        with env:
            self.robot.SetTransform(checking_pose)
        isCollision =  env.CheckCollision(self.robot)
        with env:
            self.robot.SetTransform(current_pose)
        return isCollision

    def BoundaryCheck(self, config):
        #config = self.discrete_env.GridCoordToConfiguration(coord)
        if config[0] < self.lower_limits[0] or config[1] < self.lower_limits[1] or config[2] < self.lower_limits[2]:
            return False
        if config[0] > self.upper_limits[0] or config[1] > self.upper_limits[1] or config[2] > self.upper_limits[2]:
            return False
        return True


    def ComputeDistance(self, start_id, end_id):

        dist = 0
        start_config = self.discrete_env.NodeIdToConfiguration(start_id)        
        end_config = self.discrete_env.NodeIdToConfiguration(end_id)

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
        
        start_config = self.discrete_env.NodeIdToConfiguration(start_id)        
        end_config = self.discrete_env.NodeIdToConfiguration(goal_id)
        dist = numpy.linalg.norm(start_config[0:2]-end_config[0:2]) 
        orientation = numpy.linalg.norm(start_config[-1]-end_config[-1]) 
        print "========================================="
        print "start_config = " + str(start_config)
        print "end_config = " + str(end_config)
        print "dist cost = " + str(dist)
        print "orientation cost = " + str(orientation)
        cost = dist + orientation
        
        #cost = self.ComputeDistance(start_id,goal_id)
        return cost

