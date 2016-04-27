import numpy
from DiscreteEnvironment import DiscreteEnvironment
import math
import IPython
import time

class HerbEnvironment(object):
    
    def __init__(self, herb, resolution):
        
        self.robot = herb.robot
        self.lower_limits, self.upper_limits = self.robot.GetActiveDOFLimits()
        self.discrete_env = DiscreteEnvironment(resolution, self.lower_limits, self.upper_limits)

        # account for the fact that snapping to the middle of the grid cell may put us over our
        #  upper limit
        upper_coord = [x - 1 for x in self.discrete_env.num_cells]
        upper_config = self.discrete_env.GridCoordToConfiguration(upper_coord)
        for idx in range(len(upper_config)):
            self.discrete_env.num_cells[idx] -= 1

        # add a table and move the robot into place
        table = self.robot.GetEnv().ReadKinBodyXMLFile('models/objects/table.kinbody.xml')
        
        self.robot.GetEnv().Add(table)

        table_pose = numpy.array([[ 0, 0, -1, 0.7], 
                                  [-1, 0,  0, 0], 
                                  [ 0, 1,  0, 0], 
                                  [ 0, 0,  0, 1]])
        table.SetTransform(table_pose)
        
        self.table = table
        # set the camera
        camera_pose = numpy.array([[ 0.3259757 ,  0.31990565, -0.88960678,  2.84039211],
                                   [ 0.94516159, -0.0901412 ,  0.31391738, -0.87847549],
                                   [ 0.02023372, -0.9431516 , -0.33174637,  1.61502194],
                                   [ 0.        ,  0.        ,  0.        ,  1.        ]])
        self.robot.GetEnv().GetViewer().SetCamera(camera_pose)
        self.p = 0.2
    
    def GetSuccessors(self, node_id):

        successors = []

        # TODO: Here you will implement a function that looks
        #  up the configuration associated with the particular node_id
        #  and return a list of node_ids that represent the neighboring
        #  nodes
        coord = self.discrete_env.NodeIdToGridCoord(node_id)
        if not self.BoundaryCheck(coord) or self.CollisionCheck(coord):
            #print "Failed to find successors"
            return successors

        successors = self.NeighborCheck(coord)
        
        return successors 

    def CollisionCheck(self,coord):
        config = self.discrete_env.GridCoordToConfiguration(coord)

        env = self.robot.GetEnv()
        jvalues = self.robot.GetDOFValues()
        jvalues[0:7] = config # 
        with env:
            self.robot.SetDOFValues(jvalues)
        isCollision =  env.CheckCollision(self.robot)
        return isCollision


    def BoundaryCheck(self, coord):
        config = self.discrete_env.GridCoordToConfiguration(coord)
        product = True
        for i in range(0,len(config)):
            product * (config[i] > self.lower_limits[i] and config[i] < self.upper_limits[i])
        #print "boundary check: " + str(product)
        return product

    def NeighborCheck(self, coord):
        neighbors = []
        for i in range(0, len(coord)):
            new_coord = list(coord)
            new_coord[i] = new_coord[i] + 1
            #print new_coord, coord
            neighbors.append(new_coord)

            new_coord = list(coord)
            new_coord[i] -= 1
            neighbors.append(new_coord)

        successors = []

        for i in xrange(len(neighbors)):
            if not self.CollisionCheck(neighbors[i]) and self.BoundaryCheck(neighbors[i]):
                #print "appending successor"
                successors.append(self.discrete_env.GridCoordToNodeId(neighbors[i]))
        return successors 

    def ComputeDistance(self, start_id, end_id):

        dist = 0

        # TODO: Here you will implement a function that 
        # computes the distance between the configurations given
        # by the two node ids
        
        start_config = self.discrete_env.NodeIdToConfiguration(start_id)        

        end_config = self.discrete_env.NodeIdToConfiguration(end_id)

        #print "start_config: " + str(start_config)
        #print "end_config: " + str(end_config) 
        total = 0
        for i in range(0, len(start_config)):
            dx = start_config[i] - end_config[i]
            total += dx**2
        dist = math.sqrt(total)
        #print "dist: " + str(dist)
        return dist

    def ComputeConfigDistance(self, start_config, end_config):
        
        #
        # TODO: Implement a function which computes the distance between
        # two configurations
        #
        dist = numpy.linalg.norm(start_config-end_config)       

        return dist


    def ComputeHeuristicCost(self, start_id, goal_id):
        
        cost = 0

        # TODO: Here you will implement a function that 
        # computes the heuristic cost between the configurations
        # given by the two node ids
        cost = self.ComputeDistance(start_id, goal_id)
        return cost

    #RRT function
    def SetGoalParameters(self, goal_config, p = 0.2):
        self.goal_config = goal_config
        self.p = p
        

    def GenerateRandomConfiguration(self):
        config = [0] * len(self.robot.GetActiveDOFIndices())
        #print self.robot.GetActiveDOFIndices()
        
        legalConfig = False
        while (not legalConfig):
            # Generate random config
            #config=numpy.random.rand(len(self.robot.GetActiveDOFIndices()))
            #config=config - .5
            #config = 4 * config 
            # Set joints to random config
            lower, upper = self.robot.GetActiveDOFLimits()
            for i in range(7):
                config[i] = numpy.random.uniform(lower[i], upper[i])
            #pdb.set_trace()
            self.robot.SetDOFValues(config,self.robot.GetActiveDOFIndices(),checklimits=1)
            # Check for collisions
            # If legal exit loop
            selfC = self.robot.CheckSelfCollision()
            env = self.robot.GetEnv()
            envC = env.CheckCollision(self.robot,self.table)

            # print "Self C: " + str(selfC) + " env C: " + str(envC) 
            if ((not selfC) and (not envC)):
                legalConfig=True

        return numpy.array(config)

    def ExtendN(self, start_config, end_config):
        
        #
        # TODO: Implement a function which attempts to extend from 
        #   a start configuration to a goal configuration
        #

        increment_length = 0.01
        step_size = 0.3
        
        current_position = numpy.array([0,0,0,0,0,0,0])

        dist = self.ComputeDistance(start_config,end_config)
        unit_vector = (end_config - start_config)/dist
        increment_dist = unit_vector*increment_length
        interpolate_num = int(dist/increment_length)

        for i in range(interpolate_num):
            
            config = start_config + increment_dist*(i+1)
            #pdb.set_trace()
            self.robot.SetDOFValues(config,self.robot.GetActiveDOFIndices(),checklimits=1)

            selfC = self.robot.CheckSelfCollision()
            env = self.robot.GetEnv()
            envC = env.CheckCollision(self.robot,self.table)

            if ((selfC) or (envC)):
                #One can either move until obstacle (CONNECT), or just discard it (EXTEND)
                #return current_position - increment_dist
                self.robot.SetDOFValues(start_config,self.robot.GetActiveDOFIndices(),checklimits=1)
                return None
        #if numpy.linalg.norm(current_position - start_config)> step_size:
        #    return start_config + step_size*unit_vector
        #else:
        return end_config

    def Extend(self, start_config, end_config):
        
        #
        # TODO: Implement a function which attempts to extend from 
        #   a start configuration to a goal configuration
        #

        increment_length = 0.01
        step_size = 0.3
        
        current_position = numpy.array([0,0,0,0,0,0,0])

        dist = self.ComputeConfigDistance(start_config,end_config)
        unit_vector = (end_config - start_config)/dist
        increment_dist = unit_vector*increment_length
        interpolate_num = int(dist/increment_length)

        for i in range(interpolate_num):
            
            config = start_config + increment_dist*(i+1)
            #pdb.set_trace()
            self.robot.SetDOFValues(config,self.robot.GetActiveDOFIndices(),checklimits=1)

            selfC = self.robot.CheckSelfCollision()
            env = self.robot.GetEnv()
            envC = env.CheckCollision(self.robot,self.table)

            if ((selfC) or (envC)):
                #One can either move until obstacle (CONNECT), or just discard it (EXTEND)
                #return current_position - increment_dist
                self.robot.SetDOFValues(start_config,self.robot.GetActiveDOFIndices(),checklimits=1)
                return None
        #if numpy.linalg.norm(current_position - start_config)> step_size:
        #    return start_config + step_size*unit_vector
        #else:
        return end_config

    def ShortenPath(self, path, timeout=5.0):
        
        start = time.time()
        while (time.time()-start)<50:
            increment_length = 0.1
            goal_config = path[-1]
            l = len(path)
            new_path = []
            new_path.append(path[0])
            for i in range(l):
                end_config = path[i+1]
                start_config = path[i] 
                dist = self.ComputeConfigDistance(start_config,end_config)
                unit_vector = (end_config - start_config)/dist
                increment_dist = unit_vector*increment_length
                interpolate_num = int(dist/increment_length)
                new_path.append(path[i])
                for j in range(interpolate_num):
                    current_position = start_config + increment_dist*(j+1)
                    check = self.ExtendN(current_position,goal_config)

                    if check != None:
                        #pdb.set_trace()
                        new_path.append(current_position)
                        new_path.append(goal_config)
                        break
                else:
                    continue
                break
            return new_path
        return path 

