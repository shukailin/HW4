import numpy, operator
from RRTPlanner import RRTTree
from SimpleEnvironment import SimpleEnvironment

import pdb

class RRTConnectPlanner(object):

    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize

    def Plan(self, start_config, goal_config, epsilon = 0.01):
        start_config = numpy.copy(start_config)
        goal_config = numpy.copy(goal_config)
        self.ftree = RRTTree(self.planning_env, start_config)
        self.rtree = RRTTree(self.planning_env, goal_config)
        plan = []

        #pdb.set_trace()

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)
        # TODO: Here you will implement the rrt connect planner
        #  The return path should be an array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space
        self.ftree.AddEdge(self.ftree.GetRootId, self.ftree.GetRootId)
        self.rtree.AddEdge(self.rtree.GetRootId, self.rtree.GetRootId)
        q_nearest_f = start_config
        q_nearest_r = goal_config

        goal_index_f = 0
        goal_index_r = 0
        
        while(self.planning_env.ComputeConfigDistance(q_nearest_f,q_nearest_r) > epsilon):
            
            #pdb.set_trace()

            q_target = self.planning_env.GenerateRandomConfiguration()

            vid_f, q_nearest_f = self.ftree.GetNearestVertex(q_target)
            vid_r, q_nearest_r = self.rtree.GetNearestVertex(q_target)


            q_extended_f = self.planning_env.ExtendN(q_nearest_f,q_target)
            q_extended_r = self.planning_env.ExtendN(q_nearest_r,q_target)

            if q_extended_f is not None:
                if numpy.array_equal(q_nearest_f,q_extended_f) == False:
                    self.ftree.AddVertex(q_extended_f)
                    self.ftree.AddEdge(vid_f,len(self.ftree.vertices)-1)
                
                    #self.planning_env.PlotEdge(q_nearest_f,q_extended_f)
            if q_extended_r is not None:
                if numpy.array_equal(q_nearest_r,q_extended_r) == False:
                    self.rtree.AddVertex(q_extended_r)
                    self.rtree.AddEdge(vid_r,len(self.rtree.vertices)-1)
                
                    #self.planning_env.PlotEdge(q_nearest_r,q_extended_r)
            
            if q_extended_f is not None and q_extended_r is not None:
                if numpy.array_equal(q_extended_f, q_extended_r):
                    goal_index_f = len(self.ftree.vertices)-1
                    goal_index_r = len(self.rtree.vertices)-1
                    break      
          
        #build tree
        current_index_f = goal_index_f
        current_index_r = goal_index_r

        while current_index_f != 0:
            plan.append(self.ftree.vertices[current_index_f])
            current_index_f = self.ftree.edges[current_index_f]
        plan.append(self.ftree.vertices[current_index_f])
        plan = plan[::-1]

        while current_index_r != 0:
            plan.append(self.rtree.vertices[current_index_r])
            current_index_r = self.rtree.edges[current_index_r]
        plan.append(self.rtree.vertices[current_index_r])
        print "Tree Size: " + str(len(self.rtree.vertices) + len(self.ftree.vertices))
        return plan
