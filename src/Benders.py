import time
from src import Params
from src import Network
from src import Zone
from src import Link
from docplex.mp.model import Model
import math


class Benders:
    
    def __init__(self, network, max_cost_):
        self.network = network
        
        self.max_cost = max_cost_
        
        
        self.params = network.params
        
        print("max obj", self.maxObj())
     
    def milp(self):
        print("B", self.network.B)
        
        t_total = time.time()
        
        self.milp = Model()
        
        self.milp.z = {(r,s):self.milp.integer_var(lb=0, ub=1) for (r,s) in self.possible}
        
        
        
        self.milp.x = {(r,s) : dict() for (r,s) in self.possible}
        
        for (r,s) in self.possible:
            self.milp.x[(r,s)] = {a: self.milp.integer_var(lb=0, ub=1) for a in self.network.links}
            for a in self.network.links:
                if a.enabled == False:
                    self.milp.x[(r,s)][a].ub = 0
                        
        self.milp.y = {a:self.milp.integer_var(lb=0, ub=1) for a in self.network.candidates}
        
        for a in self.network.candidates:
            if a.start.id < a.end.id:
                self.milp.add_constraint(self.milp.y[a] == self.milp.y[self.network.findLink(a.end, a.start)])
        
        if self.max_cost > 1000:
            M = 1e7
        else:
            M = 1e4
        
        self.milp.dummy = {(r,s):self.milp.integer_var(lb=0, ub=1) for r in self.network.origins for s in r.getDests()}
        
        for (r,s) in self.possible:
            if r != s:
                self.milp.add_constraint(self.milp.dummy[(r,s)] * (self.max_cost+1) + sum(self.milp.x[(r,s)][a] * a.t_ff for a in self.network.links) <= self.max_cost + M* (1-self.milp.z[(r,s)]))
            else:
                self.milp.add_constraint(self.milp.z[(r,s)] == 1)
                self.milp.add_constraint(self.milp.dummy[(r,s)] == 1)
                
            for j in self.network.nodes:
                d = 0
                if j == r:
                    d = -1
                    self.milp.add_constraint(sum(self.milp.x[(r,s)][ij] for ij in j.incoming) - self.milp.dummy[(r,s)] - sum(self.milp.x[(r,s)][jk] for jk in j.outgoing)  == d)
                elif j == s:
                    d = 1
                    self.milp.add_constraint(sum(self.milp.x[(r,s)][ij] for ij in j.incoming) + self.milp.dummy[(r,s)] - sum(self.milp.x[(r,s)][jk] for jk in j.outgoing) == d)
                else:
                    d = 0
                    self.milp.add_constraint(sum(self.milp.x[(r,s)][ij] for ij in j.incoming) - sum(self.milp.x[(r,s)][jk] for jk in j.outgoing) == d)
                
                
    
    
            for a in self.network.candidates:
                self.milp.add_constraint(self.milp.x[(r,s)][a] <= self.milp.y[a])
        
        self.milp.maximize(sum(self.milp.z[(r,s)] * r.getDemand(s) for (r,s) in self.possible))
        
        
        self.milp.add_constraint(sum(self.milp.y[a] for a in self.network.candidates) <= 2*self.network.B)
        
        self.milp.set_time_limit(self.params.max_time)
        self.milp.parameters.mip.tolerances.mipgap = self.params.min_gap
        
        self.milp.solve(log_output=False)
        y = {a:self.milp.y[a].solution_value for a in self.network.candidates}
        
        details = self.milp.get_solve_details()
        gap = details.mip_relative_gap
        
        self.z_milp = {(r,s): self.milp.z[(r,s)].solution_value for (r,s) in self.possible}
        
        for a in y:
            a.y = y[a]
            
        
        
        print("check obj", self.calcObjZ(self.z_milp), self.calcObj(y))
        
        #print(y)
        
 
        '''
        for (r,s) in self.z_milp:
            if self.z_milp[(r,s)] == 1:
                print((r,s), self.z_milp[(r,s)], r.getDemand(s))
                
                lista = list()
                for a in self.network.links:
                    if self.milp.x[(r,s)][a].solution_value > 0.1:
                        if a in self.network.candidates:
                            msg = a.y
                        elif a.enabled == True:
                            msg = True
                        
                        #if r.id == 19 and s.id == 17:
                        print("\t", a, self.milp.x[(r,s)][a].solution_value, a.t_ff, msg)
            
        '''      
        
        obj = self.milp.objective_value
        
        self.obj_milp = obj
        
        self.y_milp = y
        t_total = time.time() - t_total
        
        print(obj, t_total)
 
        return y, obj, gap, t_total
    
    def getNumSelected(self, y):
        output = 0
        for a in y:
            if y[a] > 0.1:
                output += 1
                
        return output
    
        
    def compare(self):
        print("\nMILP/CPlex")
        y_milp, obj_milp, gap_milp, t_milp = self.milp()
        
        print("\nBender's")
        y_bd, obj_bd, gap_bd, t_bd, iter_bd = self.benders()
        
        if obj_bd > -0.01:
            obj_bd = max(0, obj_bd)
            
        
        print("\nBhagat")
        y_bhagat, obj_bhagat, t_bhagat = self.bhagat()
        
        print("\n")
        
        max_obj = max(obj_milp, obj_bd)
        gap_bhagat = (max_obj - obj_bhagat) / obj_bhagat
        
        print("", "obj", "gap", "cpu time", "num selected", "iter")
        print("MILP", obj_milp, gap_milp, t_milp, self.getNumSelected(y_milp)/2)
        #print("\t", y_milp)
        print("BD", obj_bd, gap_bd, t_bd, self.getNumSelected(y_bd)/2, iter_bd)
        print("Bhagat", obj_bhagat, gap_bhagat, t_bhagat, self.getNumSelected(y_bhagat)/2)
        
        print("\n\n")
        
        
        print(round(len(self.network.candidates)/2), "&", self.max_cost, "&", round(obj_bd), "&", round(gap_bd, 2), "&", round(t_bd, 1), "&", iter_bd, "&", round(obj_milp), "&", round(gap_milp, 2), "&", round(t_milp,1), "&", round(obj_bhagat), "&", round(gap_bhagat, 2), "&", round(t_bhagat, 1), "\\\\\n")
        
        '''
        for (r,s) in self.z_milp:
            if round(self.z_milp[(r,s)]) != round(self.z_bd[(r,s)]):
                
                tot_length = 0
                for a in self.network.links:
                    if self.milp.x[(r,s)][a].solution_value > 0.1:
                        tot_length += a.t_ff
                
                print((r,s), self.z_milp[(r,s)], self.z_bd[(r,s)], tot_length, self.max_cost, r.getDemand(s))
                
                for a in self.network.links:
                    if self.milp.x[(r,s)][a].solution_value > 0.1:
                        if a in self.network.candidates:
                            msg = a.y
                        elif a.enabled == True:
                            msg = True
                        
                        #if r.id == 19 and s.id == 17:
                        #    print("\t", a, self.milp.x[(r,s)][a].solution_value, a.t_ff, msg)
        '''
               
    def initRMP(self):
        
        for a in self.network.links:
            a.y = 1
            
        for a in self.network.candidates:
            a.y = 0
            
            
        self.rmp = Model()
        
        self.rmp.zeta = {(r,s):self.rmp.continuous_var(lb=0, ub=1) for (r,s) in self.possible}
        
        self.rmp.y = {a: self.rmp.integer_var(lb=0,ub=1) for a in self.network.candidates}
        
        for a in self.network.candidates:
            if a.start.id < a.end.id:
                self.rmp.add_constraint(self.rmp.y[a] == self.rmp.y[self.network.findLink(a.end, a.start)])
        
        self.rmp.add_constraint(sum(self.rmp.y[a] for a in self.network.candidates) <= 2*self.network.B)
        
        self.rmp.maximize(sum(self.rmp.zeta[(r,s)] * r.getDemand(s) for (r,s) in self.possible))
        
        
        
     
    def initLinkMu(self):
        self.linkMu = dict()
        
        # I want to assume that links are bidirectional
        
        for (r,s) in self.possible:
            self.linkMu[(r,s)] = dict()
            
            self.network.dijkstras(r, self.max_cost, True)
            
            
            nodeCostTo = {j: j.cost for j in self.network.nodes}
            
            self.network.dijkstrasTo(s, self.max_cost, True)
            
            nodeCostFrom = {j: j.cost for j in self.network.nodes}
            
            #if r.id == 2 and s.id == 6:
            #    print(nodeCostFrom)
            
            for a in self.network.candidates:
                i = a.start
                j = a.end
                
                
                
                    
                    
                        
                if nodeCostTo[i] + nodeCostFrom[j] + a.t_ff <= self.max_cost:
                    self.linkMu[(r,s)][a] = 1
                else:
                    self.linkMu[(r,s)][a] = 0
                
                '''
                if r.id == 19 and s.id == 17:
                    print("link mu calc", (r,s), a, nodeCostTo[i], nodeCostFrom[j], a.t_ff, self.linkMu[(r,s)][a])
                '''
        '''
        for (r,s) in self.possible:
            if r.id == 19 and s.id == 17:
                print("link mu", self.linkMu[(r,s)])
        '''
        
    def maxObj(self):
        output = 0
        self.possible = set()
        for r in self.network.origins:
            self.network.dijkstras(r, self.max_cost, True) 
        
            for s in r.getDests():
                if s.cost <= self.max_cost and r.getDemand(s) > 0.001:
                    output += r.getDemand(s)
                    self.possible.add((r,s))
                    
        return output
              
    def benders(self):
        
        t_init = time.time()
        t_total = time.time()
        
        self.initLinkMu()
        
        t_init = time.time() - t_init
        
        
        
        self.initRMP()
        
        print("finished init")
        
        lb = 0
        ub = 1e15
        
        besty = dict()
        
        gap = 100
        
        iteration = 0
        
        while gap > 0.01:
            iteration += 1
            y, obj, z = self.solveRMP()
            
            ub = obj
            
            valid_obj = self.subproblem(y)
            
            
            if valid_obj > lb:
                besty = y
                self.z_bd = z
                lb = valid_obj

            if lb > 0:
                gap = (ub - lb)/lb
            
            
            
            
              
            if ub < self.obj_milp-0.1:
                for a in self.network.candidates:
                    self.rmp.add_constraint(self.rmp.y[a] == self.y_milp[a])
                print("old ub", ub, "solving rmp")

                y, obj, z = self.solveRMP()
                print("new obj", obj)
                
                #print("check obj", self.calcObjZ(z))
                
                for (r,s) in self.possible:
                    if z[(r,s)] > 0.1:
                        print((r,s), z[(r,s)], r.getDemand(s))
                
                print("exit due to BD ub < MILP obj")
                break
            
            time_elapse = time.time() - t_total
            
            
            
            print(iteration, lb, ub, gap, time_elapse)
            
            if gap <= self.params.min_gap:
                break
                
            if time_elapse >= self.params.max_time:
                break
                
        t_total = time.time() - t_total
        
        
        print("validate", self.calcObj(besty))
        
        return besty, lb, gap, t_total+t_init, iteration
     
    def solveRMP(self):
        self.rmp.solve(log_output=False)
        y = {a:self.rmp.y[a].solution_value for a in self.network.candidates}
        obj = self.rmp.objective_value
        
        #z = dict()
        z = {(r,s): self.rmp.zeta[(r,s)].solution_value for (r,s) in self.possible}
        
        return y, obj, z
           
    def subproblem(self, y):
          
        for a in y:
            a.y = round(y[a])
            
        obj = 0
            
        for r in self.network.origins:
            self.network.dijkstras(r, self.max_cost, False)
            
            for s in r.getDests():
                
                if (r,s) in self.possible:
                    gamma_rs = 0
                    
                    if s.cost <= self.max_cost:
                        gamma_rs = 1
                        
                    obj += gamma_rs * r.getDemand(s)
                    
                    mu = dict()
                    for a in self.network.candidates:               
                        if y[a] < 1e-2:
                            mu[a] = self.linkMu[(r,s)][a]
                        else:
                            mu[a] = 0
                        
                    
                    self.rmp.add_constraint(self.rmp.zeta[(r,s)] <= gamma_rs + sum(self.rmp.y[a] * mu[a] for a in self.network.candidates))
                    
                    '''
                    if r.id == 19 and s.id == 17:
                        print("y", y)
                        print("link mu", self.linkMu[(r,s)])
                        print("gamma", gamma_rs)
                        for a in mu:
                            print("\tmu", a, mu[a], y[a], self.linkMu[(r,s)][a])
                    '''
                
                
        return obj
    
    def calcObj(self, y):
        for a in self.network.links:
            a.y = 1
            
        for a in self.network.candidates:
            a.y = round(y[a])
        
        
        output = 0
        for r in self.network.origins:
            for s in r.getDests():
                self.network.dijkstras(r, self.max_cost, False)
                if s.cost <= self.max_cost:
                
                    output += r.getDemand(s)
                    
                    #print( (r,s), 1)
        
        return output
        
    def calcObjZ(self, z):
        output = 0
        for (r,s) in z:
            if z[(r,s)] > 0.1:
                output += r.getDemand(s)
                
        return output
        
        
    def bhagat(self):
        
        t_total = time.time()
        
        budget_used = 0
        
        for a in self.network.candidates:
            a.y = 0
        
        best_link = None
        best_score = 0
        
        while budget_used < self.network.B:
            
            
            old_dist = dict()
            
            for r in self.network.origins:
                self.network.dijkstras(r, self.max_cost, False)
                
                for s in r.getDests():
                    if (r,s) in self.possible:
                        old_dist[(r,s)] = s.cost
                        
            
            
            for a in self.network.candidates:
                if a.y == 0:
                    i = a.start
                    j = a.end
                    
                    new_dist = dict()
                    
                    self.network.dijkstrasTo(i, self.max_cost, False)
                    
                    to_costs = {n : n.cost for n in self.network.nodes}
                    
                    self.network.dijkstras(j, self.max_cost, False)
                    
                    score = 0
                    
                    for (r,s) in self.possible:
                        new_dist[(r,s)] = to_costs[r] + a.t_ff + s.cost
                    
                        
                        
                        old_access = 0
                        new_access = 0
                        
                        if old_dist[(r,s)] <= self.max_cost:
                            old_access = 1
                            new_access = 1
                        if new_dist[(r,s)] <= self.max_cost:
                            new_access = 1
                            
                            
                        score += r.getDemand(s) * (new_access - old_access)
                    
                    
                    if score > best_score:
                        best_link = a
            
            if best_link is None:
                print("Could not find best link")
                for a in self.network.candidates:
                    if a.y == 0:
                        print("\t", a, a.y) 
                break
            else:            
                best_link.y = 1
                self.network.findLink(best_link.end, best_link.start).y = 1
                budget_used += 1
                
                best_score = 0
                best_link = None
                
        y = {a : a.y for a in self.network.candidates}   
        
        self.y_bhagat = y
        
        
        
        obj = self.calcObj(y)     
                
        t_total = time.time() - t_total
        
        return y, obj, t_total