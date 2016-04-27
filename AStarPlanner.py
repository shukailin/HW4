import IPython

class AStarPlanner(object):
    
    def __init__(self, planning_env, visualize):
        self.planning_env = planning_env
        self.visualize = visualize
        self.nodes = dict()


    def Plan(self, start_config, goal_config):
        print "insider Astar"
        plan = []
        
        # TODO: Here you will implement the AStar planner
        #  The return path should be a numpy array
        #  of dimension k x n where k is the number of waypoints
        #  and n is the dimension of the robots configuration space

        if self.visualize and hasattr(self.planning_env, 'InitializePlot'):
            self.planning_env.InitializePlot(goal_config)

        start_node_id = self.planning_env.discrete_env.ConfigurationToNodeId(start_config)
        goal_node_id = self.planning_env.discrete_env.ConfigurationToNodeId(goal_config)
        
        closed_set = set()
        open_set   = set()
        open_set.add(start_node_id)
        node_map = dict()

        current = start_node_id
        g_cost = {start_node_id:0}
        h_cost = {start_node_id: self.planning_env.ComputeHeuristicCost(start_node_id,goal_node_id)}
        f_cost = {start_node_id:g_cost[start_node_id]+h_cost[start_node_id]}
        total_num = 0
        
        while len(open_set) != 0:
            min_node   = 0
            min_f_cost = float(1000)
            #assigning lowest f_cost value to the current node
            for node in open_set: 
                if f_cost[node] < min_f_cost:
                    min_node = node
                    min_f_cost = f_cost[node]
            #print "expanding: " + str(min_node) + " with F cost: " + str(f_cost[min_node])
            #print "current jvs: " + str(self.planning_env.discrete_env.NodeIdToConfiguration(current))
            
            current = min_node
            
            open_set.remove(current)
            closed_set.add(current)
            #print "debug 1"
            #if solution is found, backtrack the path, and return it
            if(current == goal_node_id):
                print "Solution found"
                n = current
                while(n != start_node_id):
                    plan.insert(0, self.planning_env.discrete_env.NodeIdToConfiguration(n))
                    #print str(n) + ", " + str(self.planning_env.discrete_env.NodeIdToGridCoord(n))
                    #raw_input()
                    n = node_map[n]
                self.node_count = len(closed_set)
                #print "debug plan"
                print "Number of visitied nodes: " + str(total_num)
                return plan
                break


            
            #print "debug 2"
            #find the nearest neighbors
            for neighbor in self.planning_env.GetSuccessors(current):
                #print "neighbor: " + str(neighbor) + " coords: " + str(self.planning_env.discrete_env.NodeIdToGridCoord(neighbor)) +  " (F, G, H) " + str(f_cost[neighbor]) + " , " + str(g_cost[neighbor])+ " , " + str(h_cost[neighbor])
                if(neighbor in closed_set or neighbor in open_set):
                    continue
                else:
                    if self.visualize:
                        self.planning_env.PlotEdge(
                            self.planning_env.discrete_env.NodeIdToConfiguration(current),
                            self.planning_env.discrete_env.NodeIdToConfiguration(neighbor))
                    total_num +=1
                    open_set.add(neighbor)
                    g_cost[neighbor] = self.planning_env.ComputeDistance(current,neighbor)
                    h_cost[neighbor] = self.planning_env.ComputeHeuristicCost(neighbor,goal_node_id)
                    f_cost[neighbor] = g_cost[neighbor] + h_cost[neighbor]
                    node_map[neighbor] = current
                   
            
            #raw_input()
            #temp_g_cost = g_cost[current] + self.planning_env.ComputeDistance(current,neighbor

            '''
            if (not (neighbor in open_set) or temp_g_cost < g_cost[neighbor]):
                node_map[neighbor] = current
                if self.visualize:
                    self.planning_env.PlotEdge(self.planning_env.discrete_env.NodeIdToConfiguration(current),self.planning_env.discrete_env.NodeIdToConfiguration(neighbor))
                g_cost[neighbor] = temp_g_cost
                f_cost[neighbor] = g_cost[neighbor] + self.planning_env.ComputeHeuristicCost(neighbor,goal_node_id)
                
                if not (neighbor in open_set):
                    open_set.add(neighbor)

            '''