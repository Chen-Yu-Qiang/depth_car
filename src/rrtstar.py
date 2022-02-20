import numpy as np
import matplotlib
matplotlib.use('TKAgg')
from matplotlib import pyplot as plt
import viewplan
import TREEDATA
import matplotlib.cm as cm

WEIGH_DIS=1
WEIGH_TH=0
WEIGH_VP=10
File_NAME="rrt_20220127_12-11-38"
TEST_NAME=File_NAME+"-("+str(WEIGH_DIS)+","+str(WEIGH_TH)+","+str(WEIGH_VP)+")"


def plot_scene(obstacle_list, start, goal):
    ax = plt.gca()
    for o in obstacle_list:
        circle = plt.Circle((o[0], o[1]), o[2], color='k')
        ax.add_artist(circle)
    plt.axis([bounds[0], bounds[1], bounds[2], bounds[3]])
    plt.plot(start[0], start[1], "xr", markersize=10)
    plt.plot(goal[0], goal[1], "xb", markersize=10)
    plt.legend(('Start', 'Goal'),loc='lower right')
    plt.gca().set_aspect('equal')
    
class RRT:
 
    class Node:
        def __init__(self, p):
            self.p = np.array(p)
            self.ang=np.pi*0.3
            self.parent = None
            self.ind=0

    def __init__(self, start, goal, obstacle_list, bounds, 
                 max_extend_length=0.5, path_resolution=0.5, 
                 goal_sample_rate=0.05, max_iter=500, treedata=[]):
        self.start = self.Node(start)
        self.goal = self.Node(goal)
        self.bounds = bounds
        self.max_extend_length = max_extend_length
        self.path_resolution = path_resolution
        self.goal_sample_rate = goal_sample_rate
        self.max_iter = max_iter
        self.obstacle_list = obstacle_list
        self.node_list = []
        self.max_ang=np.pi*0.5
        self.treedata=treedata
        self.final_path_list=[]

    def plan(self):
        """Plans the path from start to goal while avoiding obstacles"""
        self.node_list = [self.start]
        for i in range(self.max_iter):
            # modify here: 
            # 1) Create a random node (rnd_node) inside 
            # the bounded environment
            rnd_node=self.get_random_node()


            # 2) Find nearest node (nearest_node)
            nearest_node=self.get_nearest_node(self.node_list,rnd_node)

            # 3) Get new node (new_node) by connecting
            # rnd_node and nearest_node. Hint: steer
            new_node=self.steer(nearest_node,rnd_node)
            
            # 4) If the path between new_node and the
            # nearest node is not in collision, add it to the node_list
            if self.collision(new_node, nearest_node, self.obstacle_list):
                pass
            else:
                self.node_list.append(new_node)

            # Don't need to modify beyond here
            # If the new_node is very close to the goal, connect it
            # directly to the goal and return the final path
            if self.dist_to_goal(self.node_list[-1].p) <= self.max_extend_length:
                final_node = self.steer(self.node_list[-1], self.goal, self.max_extend_length)
                if not self.collision(final_node, self.node_list[-1], self.obstacle_list):
                    return self.final_path(len(self.node_list) - 1)
        return None  # cannot find path

    def steer(self, from_node, to_node, max_extend_length=np.inf):
        """Connects from_node to a new_node in the direction of to_node
        with maximum distance max_extend_length
        """
        new_node = self.Node(to_node.p)
        d = from_node.p - to_node.p
        dist = np.linalg.norm(d)
        if dist > max_extend_length:
            # rescale the path to the maximum extend_length
            new_node.p  = from_node.p - d / dist * max_extend_length
        new_node.parent = from_node
        return new_node

    def dist_to_goal(self, p):
        """Distance from p to goal"""
        return np.linalg.norm(p - self.goal.p)

    def get_random_node(self):
        """Sample random node inside bounds or sample goal point"""
        if np.random.rand() > self.goal_sample_rate:
            # Sample random point inside boundaries
            x=np.random.rand()*(self.bounds[1]-self.bounds[0]) + self.bounds[0]
            y=np.random.rand()*(self.bounds[3]-self.bounds[2]) + self.bounds[2]
            rnd = self.Node(np.array([x,y]))
        else:  
            # Select goal point
            rnd = self.Node(self.goal.p)
        return rnd
    
    
    def get_nearest_node(self,node_list, node):
        """Find the nearest node in node_list to node"""

        dlist = [np.sum(np.square((node.p - n.p))) for n in node_list]
        minind = dlist.index(min(dlist))
        return node_list[minind]
    
    @staticmethod
    def collision(node1, node2, obstacle_list):
        """Check whether the path connecting node1 and node2 
        is in collision with anyting from the obstacle_list
        """
        p1 = node2.p
        p2 = node1.p 
        for o in obstacle_list:
            center_circle = o[0:2]
            radius = o[2]
            d12 = p2 - p1 # the directional vector from p1 to p2
            # defines the line v(t) := p1 + d12*t going through p1=v(0) and p2=v(1)
            d1c = center_circle - p1 # the directional vector from circle to p1
            # t is where the line v(t) and the circle are closest
            # Do not divide by zero if node1.p and node2.p are the same.
            # In that case this will still check for collisions with circles
            t = d12.dot(d1c) / (d12.dot(d12) + 1e-7)
            t = max(0, min(t, 1)) # Our line segment is bounded 0<=t<=1
            d = p1 + d12*t # The point where the line segment and circle are closest
            is_collide = np.sum(np.square(center_circle-d)) < radius**2
            if is_collide:
                return True # is in collision
        return False # is not in collision


    def out_ang_constraint(self,node, new_node):
        ang=self.ang_between_parent(node, new_node)
        # print(ang)
        if ang<self.max_ang:
            return False
        else:
            return True


    def ang_between_parent(self, node, new_node):
        ang_new=self.ang_nodeA_2_nodeB(node,new_node)
        ang_old=node.ang
        ang=abs(ang_new-ang_old)
        ang=np.arccos(np.cos(ang))
        # print(ang)
        return ang
                
    @staticmethod
    def ang_nodeA_2_nodeB(nodeA,nodeB):
      
        p1 = nodeB.p[0]
        p2 = nodeB.p[1]
        p3 = nodeA.p[0]
        p4 = nodeA.p[1]
        return np.arctan2(p2-p4,p1-p3) 

    @staticmethod
    def ang_from_parent(node1):
        p1 = node1.p[0]
        p2 = node1.p[1]
        if node1.parent is None:
            return np.pi
        else:
            p3 = node1.parent.p[0]
            p4 = node1.parent.p[1]
            return np.arctan2(p2-p4,p1-p3) 


    def final_path(self, goal_ind):
        """Compute the final path from the goal node to the start node"""
        self.goal.parent=self.node_list[goal_ind]
        self.goal.ang=self.ang_from_parent(self.goal)
        node = self.node_list[goal_ind]
        path = [self.goal.p]
        path.append(node.p)
        
        path_node=[self.goal]
        path_node.append(node)

        print("final_path----------")
        i=0
        while not node==self.start:
            node = node.parent
            path.append(node.p)
            path_node.append(node)

        print("final_path---------- len= "+str(len(path_node)))        
        path_node.reverse()
        self.final_path_list=path_node
        # for i in range(1,len(path_node)):
        #     to_node=path_node[i]
        #     from_node=path_node[i].parent
        #     # viewplan.plot_contourf(self.treedata,z=0,th=to_node.ang-0.5*np.pi,x_min=TREEDATA.X_MIN,x_max=TREEDATA.X_MAX,y_min=TREEDATA.Y_MIN,y_max=TREEDATA.Y_MAX)

        #     for j in range(1,len(path_node)):
        #         if j==i:
        #             plt.plot([path_node[j].p[0], path_node[j].parent.p[0]], [path_node[j].p[1], path_node[j].parent.p[1]], "-r")
        #         else:
        #             plt.plot([path_node[j].p[0], path_node[j].parent.p[0]], [path_node[j].p[1], path_node[j].parent.p[1]], "-g")
            
            
        #     d_s=np.linalg.norm(from_node.p - to_node.p)

        #     ang_forward=self.ang_between_parent(from_node,to_node)
        #     # ang_backward=np.pi-ang_forward
        #     # ang_s = min(ang_forward,ang_backward)*10
        #     ang_s = ang_forward

        #     vp_s_ = viewplan.line_C_s(self.treedata, from_node.p, to_node.p, to_node.ang-0.5*np.pi)
        #     vp_s=1/(0.01+vp_s_)
        #     t="Dis= "+str(round(d_s,3))+" m\nAngle= "+str(int(ang_s*57.3))+" deg\nView= "+str(round(vp_s,3))
        #     plt.text(self.bounds[0],self.bounds[3]-10,t,color="r")        
        #     plt.plot(self.start.p[0], self.start.p[1], "xr", markersize=10)
        #     plt.plot(self.goal.p[0], self.goal.p[1], "xb", markersize=10)
        #     plt.savefig("rrt_20220127_12-11-38-"+str(i)+".png",dpi=600)

        #     plt.clf()
        #     print("save "+"rrt_20220127_12-11-38-"+str(i)+".png")


        print("PLAN END----------")
        # modify here: Generate the final path from the goal node to the start node.
        # We will check that path[0] == goal and path[-1] == start
        
        return path

    def draw_graph(self):
        for node in self.node_list:
            if node.parent:
                plt.plot([node.p[0], node.parent.p[0]], [node.p[1], node.parent.p[1]], "-g")

    def draw_each(self):

        path_node=self.final_path_list
        for i in range(1,len(path_node)):
            to_node=path_node[i]
            from_node=path_node[i].parent
            viewplan.plot_contourf(self.treedata,z=0,th=to_node.ang-0.5*np.pi,x_min=self.bounds[0],x_max=self.bounds[1],y_min=self.bounds[2],y_max=self.bounds[3])

            for j in range(1,len(path_node)):
                if j==i:
                    plt.plot([path_node[j].p[0], path_node[j].parent.p[0]], [path_node[j].p[1], path_node[j].parent.p[1]], "-r")
                else:
                    plt.plot([path_node[j].p[0], path_node[j].parent.p[0]], [path_node[j].p[1], path_node[j].parent.p[1]], "-g")
            
            
            d_s=np.linalg.norm(from_node.p - to_node.p)

            ang_forward=self.ang_between_parent(from_node,to_node)
            # ang_backward=np.pi-ang_forward
            # ang_s = min(ang_forward,ang_backward)*10
            ang_s = ang_forward

            vp_s_ = viewplan.line_C_s(self.treedata, from_node.p, to_node.p, to_node.ang-0.5*np.pi)
            vp_s=1/(1+vp_s_)
            t="Dis= "+str(round(d_s,3))+" m\nAngle= "+str(round(ang_s,3))+" rad\nView= "+str(round(vp_s,3))
            plt.text(self.bounds[0],self.bounds[3]-10,t,color="r")        
            plt.plot(self.start.p[0], self.start.p[1], "xr", markersize=10)
            plt.plot(self.goal.p[0], self.goal.p[1], "xb", markersize=10)
            # plt.show()
            plt.savefig(TEST_NAME+"("+str(i)+").png",dpi=600)

            plt.clf()
            print("save "+TEST_NAME+"("+str(i)+").png")





class RRTStar(RRT):
    
    class Node(RRT.Node):
        def __init__(self, p):
            RRT.Node.__init__(self,p)
            self.cost = 0.0

    def __init__(self, start, goal, obstacle_list, bounds,
                 max_extend_length=10.0,
                 path_resolution=0.5,
                 goal_sample_rate=0.1,
                 max_iter=1000,
                 connect_circle_dist=50.0,
                 treedata=[]):
        RRT.__init__(self,start, goal, obstacle_list, bounds, max_extend_length,
                         path_resolution, goal_sample_rate, max_iter,treedata)
        self.connect_circle_dist = connect_circle_dist
        self.goal = self.Node(goal)

    def plan(self):
        """Plans the path from start to goal while avoiding obstacles"""
        self.node_list = [self.start]
        for i in range(self.max_iter):
            # Create a random node inside the bounded environment
            print(i)
            rnd = self.get_random_node()
            # Find nearest node
            nearest_node = self.get_nearest_node(self.node_list, rnd)
            # Get new node by connecting rnd_node and nearest_node
            new_node = self.steer(nearest_node, rnd, self.max_extend_length)
            # If path between new_node and nearest node is not in collision:
            if self.collision(new_node, nearest_node, self.obstacle_list):
                print(i,"collision")
                pass
                # elif self.out_ang_constraint(nearest_node,new_node):
                #     print(i,"ang")
                #     pass
                # if 0:
                #     pass
            else:
                near_inds = self.near_nodes_inds(new_node)
                last_i=self.node_list[-1].ind+1
                new_node.ind=last_i
                # Connect the new node to the best parent in near_inds
                new_node = self.choose_parent(new_node, near_inds)
                if new_node.cost<np.inf:
                    
                    self.node_list.append(new_node)
                    # Rewire the nodes in the proximity of new_node if it improves their costs
                    self.rewire(new_node, near_inds)

        last_index, min_cost = self.best_goal_node_index()
        if last_index:
            return self.final_path(last_index), min_cost
        return None, min_cost



    def choose_parent(self, new_node, near_inds):
        """Set new_node.parent to the lowest resulting cost parent in near_inds and
        new_node.cost to the corresponding minimal cost
        """
        min_cost = np.inf
        best_near_node = None
        # modify here: Go through all near nodes and evaluate them as potential parent nodes by
        # 1) checking whether a connection would result in a collision,
        # 2) evaluating the cost of the new_node if it had that near node as a parent,
        # 3) picking the parent resulting in the lowest cost and updating
        #    the cost of the new_node to the minimum cost.
        for i in near_inds:
            if self.collision(new_node,self.node_list[i],self.obstacle_list):
                pass
            elif self.out_ang_constraint(self.node_list[i],new_node):
                pass
            else:
                n_c=self.new_cost(self.node_list[i],new_node)
                if n_c<min_cost:
                    min_cost=n_c
                    best_near_node=self.node_list[i]



        # Don't need to modify beyond here
        new_node.cost = min_cost
        new_node.parent = best_near_node
        new_node.ang=self.ang_from_parent(new_node)
        return new_node
    
    def rewire(self, new_node, near_inds):
        """Rewire near nodes to new_node if this will result in a lower cost"""
        # modify here: Go through all near nodes and check whether rewiring them
        # to the new_node would: 
        # A) Not cause a collision and
        # B) reduce their own cost.
        # If A and B are true, update the cost and parent properties of the node.
        

        for i in near_inds:
            if self.collision(self.node_list[i],new_node,self.obstacle_list):
                pass
            elif self.out_ang_constraint(new_node,self.node_list[i]):
                pass
            else:
                n_c=self.new_cost(new_node,self.node_list[i])
                if n_c<self.node_list[i].cost:
                    self.node_list[i].parent=new_node
                    self.node_list[i].cost=n_c
                    self.node_list[i].ang=self.ang_from_parent(self.node_list[i])


        # Don't need to modify beyond here
        self.propagate_cost_to_leaves(new_node)

    def best_goal_node_index(self):
        """Find the lowest cost node to the goal"""
        min_cost = np.inf
        best_goal_node_idx = None
        for i in range(len(self.node_list)):
            node = self.node_list[i]
            # Has to be in close proximity to the goal
            if self.dist_to_goal(node.p) <= self.max_extend_length:
                # Connection between node and goal needs to be collision free
                if self.collision(self.goal, node, self.obstacle_list):
                    pass
                elif self.out_ang_constraint(node,self.goal):
                    pass
                else:
                    # The final path length
                    cost = node.cost + self.dist_to_goal(node.p) 
                    if node.cost + self.dist_to_goal(node.p) < min_cost:
                        # Found better goal node!
                        min_cost = cost
                        best_goal_node_idx = i
                print(i,node.p,node.ang)
        print(best_goal_node_idx, min_cost)
        return best_goal_node_idx, min_cost

    def get_nearest_cost_node(self,node_list, node):
        """Find the nearest node in node_list to node"""
        dlist=[]
        for n in node_list:
            dlist.append(self.delta_cost(node, n))
        # dlist = [np.sum(np.square((node.p - n.p)))+self.ang_between_parent(n,node) for n in node_list]
        minind = dlist.index(min(dlist))
        return node_list[minind]


    def near_nodes_inds(self, new_node):
        """Find the nodes in close proximity to new_node"""
        nnode = len(self.node_list) + 1
        r = self.connect_circle_dist * np.sqrt((np.log(nnode) / nnode))
        dlist = [np.sum(np.square((node.p - new_node.p))) for node in self.node_list]
        near_inds=[]
        for i in range(len(dlist)):
            if dlist[i]<=r ** 2:
                near_inds.append(i)
        # near_inds = [dlist.index(i) for i in dlist if i <= r ** 2]
        return near_inds
    def delta_cost_div(self, from_node, to_node):
        """to_node's new cost if from_node were the parent"""
        if not WEIGH_DIS==0:
            d_s=np.linalg.norm(from_node.p - to_node.p)
        else:
            d_s=0
        if not WEIGH_TH==0:
            ang_forward=self.ang_between_parent(from_node,to_node)
            # ang_backward=np.pi-ang_forward
            # ang_s = min(ang_forward,ang_backward)
            ang_s = ang_forward
        else:
            ang_s=0
        if not WEIGH_VP==0:
            vp_s_ = viewplan.line_C_s(self.treedata, from_node.p, to_node.p, to_node.ang-0.5*np.pi)
            vp_s=1/(1+vp_s_)
        else:
            vp_s=0
        # print("from",from_node.ind,"To",to_node.ind,"d= ",d_s," , ang= ",ang_s,", vp= ",vp_s)
        return d_s,ang_s,vp_s

    def delta_cost(self, from_node, to_node):
        d_s,ang_s,vp_s=self.delta_cost_div(from_node, to_node)
        if vp_s<0.5:
            vp_s=0
        d=0
        d = d + d_s*WEIGH_DIS
        d = d + ang_s*WEIGH_TH
        d = d + vp_s*d_s*vp_s*d_s*WEIGH_VP
        return d        
    def new_cost(self, from_node, to_node):

        return from_node.cost + self.delta_cost(from_node, to_node)

    def propagate_cost_to_leaves(self, parent_node):
        """Recursively update the cost of the nodes"""
        for node in self.node_list:
            if node.parent == parent_node:
                node.cost = self.new_cost(parent_node, node)
                self.propagate_cost_to_leaves(node)

    def draw_all_cost(self):
        x_list=[]
        y_list=[]
        z_list=[]
        for i in self.node_list:
            x_list.append(i.p[0])
            y_list.append(i.p[1])
            z_list.append(i.cost)

        plt.scatter(x_list,y_list,c=z_list)
        plt.colorbar()

    def get_cost(self):
        import csv

        with open(TEST_NAME+".csv", 'w') as csvfile:
            writer = csv.writer(csvfile)

            for i in range(1,len(self.final_path_list)):
                from_node=self.final_path_list[i-1]
                to_node=self.final_path_list[i]

                d_s,ang_s,vp_s=self.delta_cost_div(from_node, to_node)
                print(d_s,ang_s,vp_s)
                write_data=[d_s,ang_s,vp_s,WEIGH_DIS,WEIGH_TH,WEIGH_VP,\
                                from_node.p[0],from_node.p[1],from_node.ang,from_node.ind,from_node.cost,\
                                to_node.p[0],to_node.p[1],to_node.ang,to_node.ind,to_node.cost,\
                                self.max_extend_length,self.goal_sample_rate,self.max_iter,self.connect_circle_dist\
                                    ]
                for i in range(len(write_data)):
                    write_data[i]=round(write_data[i],3)
                writer.writerow(write_data)


def treedata2BoundsAndObstacles():
    obstacles=[]
    for i in TREEDATA.TREE_DATA:
        obstacles.append(np.array([i[0], i[1], i[2]+1.3]))
    return obstacles,[TREEDATA.X_MIN, TREEDATA.X_MAX,TREEDATA.Y_MIN, TREEDATA.Y_MAX]

start = np.array([2774765, -359160]) # Start location
goal = np.array([2774795, -359140]) # Goal location

obstacles,bounds=treedata2BoundsAndObstacles()

# obstacles = [ # circles parametrized by [x, y, radius]
#         np.array([3, 3, 1]),
#         np.array([3, 5, 1]),
#         np.array([3, 7, 1]),
#         np.array([3, 9, 1]),
#         np.array([3, 11, 1]),
#         np.array([3, 13, 1]),
#         np.array([5, 13, 1]),
#         np.array([7, 13, 1]),
#         np.array([9, 13, 1]),
#         np.array([11, 13, 1]),
#         np.array([13, 13, 1])
# ] 

# bounds = [0, 16,0, 16] # x_min x_max y_min y_max

# start = np.array([1,1]) # Start location
# goal = np.array([15,15]) # Goal location

treedata=viewplan.treedata2taskPoint(TREEDATA.TREE_DATA)
treedata=[]
for i in obstacles:
    treedata.append([i[0],i[1],0,0])

# plt.show()
np.random.seed(13)
rrt_star = RRTStar(start=start,
          goal=goal,
          bounds=bounds,
          obstacle_list=obstacles,
          treedata=treedata,
          max_iter=150)
path_rrt_star, min_cost = rrt_star.plan()
rrt_star.get_cost()
plot_scene(obstacles, start, goal)
rrt_star.draw_graph()
rrt_star.draw_all_cost()
plt.savefig(TEST_NAME+"all2.png",dpi=600)
print('Minimum cost: {}'.format(min_cost))

# # Check the cost
# def path_cost(path):
#     return sum(np.linalg.norm(path[i] - path[i + 1]) for i in range(len(path) - 1))

# if path_rrt_star:
#     print('Length of the found path: {}'.format(path_cost(path_rrt_star)))




plt.figure(figsize=(6,6))
plot_scene(obstacles, start, goal)
rrt_star.draw_graph()
if path_rrt_star is None:
    print("No viable path found")
else:
    plt.plot([x for (x, y) in path_rrt_star], [y for (x, y) in path_rrt_star], '-r')


plt.savefig(TEST_NAME+"all.png",dpi=600)
print("Total node "+str(len(rrt_star.node_list)))

plt.figure(figsize=(6,6))
rrt_star.draw_each()