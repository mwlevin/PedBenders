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
            self.milp.add_constraint(self.milp.dummy[(r,s)] * (self.max_cost+1) + sum(self.milp.x[(r,s)][a] * a.t_ff for a in self.network.links) <= self.max_cost + M* (1-self.milp.z[(r,s)]))
            
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
        
        
        self.z_milp = {(r,s): self.milp.z[(r,s)].solution_value for (r,s) in self.possible}
        
        for a in y:
            a.y = y[a]
        
        print("check obj", self.calcObjZ(self.z_milp), self.calcObj(y))
        
        '''
        for (r,s) in z:
            if z[(r,s)] == 1:
                print((r,s), z[(r,s)], r.getDemand(s))
                
                lista = list()
                for a in self.network.links:
                    if self.milp.x[(r,s)][a].solution_value > 0.1:
                        if a in self.network.candidates:
                            msg = a.y
                        elif a.enabled == True:
                            msg = True
                        
                        #if r.id == 2 and s.id == 6:
                        print("\t", a, self.milp.x[(r,s)][a].solution_value, a.t_ff, msg)
        '''    
              
        
        obj = self.milp.objective_value
        
        self.y_milp = y
        t_total = time.time() - t_total
        
        print(obj, t_total)
 
        return y, obj, t_total
        
    def compare(self):
        y_milp, obj_milp, t_milp = self.milp()
        y_bd, obj_bd, t_bd = self.benders()
        
        print("MILP", obj_milp, t_milp)
        #print("\t", y_milp)
        print("BD", obj_bd, t_bd)
        
        for (r,s) in self.z_milp:
            if round(self.z_milp[(r,s)]) != round(self.z_bd[(r,s)]):
                
                tot_length = 0
                for a in self.network.links:
                    if self.milp.x[(r,s)][a].solution_value > 0.1:
                        tot_length += a.t_ff
                
                print((r,s), self.z_milp[(r,s)], self.z_bd[(r,s)], tot_length, self.max_cost)
                
                for a in self.network.links:
                    if self.milp.x[(r,s)][a].solution_value > 0.1:
                        if a in self.network.candidates:
                            msg = a.y
                        elif a.enabled == True:
                            msg = True
                        
                        #if r.id == 2 and s.id == 6:
                        #print("\t", a, self.milp.x[(r,s)][a].solution_value, a.t_ff, msg)
        
               
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
        
        for r in self.network.origins:
            
            for s in r.getDests():
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
                    
                    
                    #if r.id == 2 and s.id == 6:
                    #    print((r,s), a, nodeCostTo[i], nodeCostFrom[j], a.t_ff )
                        
                        
                            
                    if nodeCostTo[i] + nodeCostFrom[j] + a.t_ff <= self.max_cost:
                        self.linkMu[(r,s)][a] = 1
                        
                        
                    else:
                        self.linkMu[(r,s)][a] = 0
                    
                
        '''
        for r in self.network.origins:
            
            for s in r.getDests():
                self.nodeCost = dict()
                
                
                
                self.linkMu[(r,s)] = dict()
                
                
                for a in self.network.links:
                    i = a.start
                    j = a.end
                    
                    self.network.dijkstras(r, self.max_cost, True)
                    
                    ell_i = i.cost
                    
                    self.network.dijkstras(j, self.max_cost, True)
                    
                    ell_j = s.cost
                    
                    if ell_i + ell_j + a.t_ff <= self.max_cost:
                        self.linkMu[(r,s)][a] = 1
                    else:
                        self.linkMu[(r,s)][a] = 0
          '''  
        
    def maxObj(self):
        output = 0
        self.possible = set()
        for r in self.network.origins:
            self.network.dijkstras(r, self.max_cost, True) 
        
            for s in r.getDests():
                if s.cost <= self.max_cost:
                    output += r.getDemand(s)
                    self.possible.add((r,s))
                    
        return output
              
    def benders(self):
        
        t_init = time.time()
        
        self.initLinkMu()
        
        t_init = time.time() - t_init
        
        t_total = time.time()
        
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
            
            
            
            
            '''    
            if ub < 43500:
                for a in self.network.candidates:
                    self.rmp.add_constraint(self.rmp.y[a] == self.milpy[a])
                print("old ub", ub, "solving rmp")

                y, obj, z = self.solveRMP()
                print("new obj", obj)
                
                #print("check obj", self.calcObjZ(z))
                
                for r in self.network.origins:
                    for s in r.getDests():
                        if z[(r,s)] > 0.1:
                            print((r,s), z[(r,s)], r.getDemand(s))
                
            '''
            
            time_elapse = time.time() - t_total
            
            
            
            print(iteration, lb, ub, gap, time_elapse)
            
            if gap <= self.params.min_gap:
                break
                
            if time_elapse >= self.params.max_time:
                break
                
        t_total = time.time() - t_total
        
        
        print("validate", self.calcObj(besty))
        
        return besty, lb, t_total+t_init
     
    def solveRMP(self):
        self.rmp.solve(log_output=False)
        y = {a:self.rmp.y[a].solution_value for a in self.network.candidates}
        obj = self.rmp.objective_value
        
        #z = dict()
        z = {(r,s): self.rmp.zeta[(r,s)].solution_value for (r,s) in self.possible}
        
        return y, obj, z
           
    def subproblem(self, y):
          
        for a in y:
            a.y = y[a]
            
        obj = 0
            
        for r in self.network.origins:
            self.network.dijkstras(r, self.max_cost, False)
            
            for s in r.getDests():
                possible = False
                
                gamma_rs = 0
                
                if s.cost <= self.max_cost:
                    gamma_rs = 1
                    possible = True
                    
                obj += gamma_rs * r.getDemand(s)
                
                mu = dict()
                for a in self.network.candidates:
                    if y[a] < 1e-2:
                        mu[a] = self.linkMu[(r,s)][a]
                        possible = True
                    else:
                        mu[a] = 0
                    
                if possible:
                    self.rmp.add_constraint(self.rmp.zeta[(r,s)] <= gamma_rs + sum(self.rmp.y[a] * mu[a] for a in self.network.candidates))
                
                '''
                if r.id == 2 and s.id == 6:
                    print("gamma", gamma_rs)
                    for a in mu:
                        print("\tmu", a, mu[a], y[a])
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