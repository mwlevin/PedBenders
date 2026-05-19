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
     
    def milp(self):
        t_total = time.time()
        
        self.milp = Model()
        
        self.milp.z = {(r,s):self.milp.integer_var(lb=0, ub=1) for r in self.network.origins for s in r.getDests()}
        
        self.milp.x = {(r,s) : dict() for r in self.network.origins for s in r.getDests()}
        
        for r in self.network.origins:
            for s in r.getDests():
                self.milp.x[(r,s)] = {a: self.milp.integer_var(lb=0, ub=1) for a in self.network.links}
                for a in self.network.links:
                    if a.enabled == False:
                        self.milp.x[(r,s)][a].ub = 0
                        
        self.milp.y = {a:self.milp.integer_var(lb=0, ub=1) for a in self.network.candidates}
        
        M = 1e4
        
        self.milp.dummy = {(r,s):self.milp.integer_var(lb=0, ub=1) for r in self.network.origins for s in r.getDests()}
        
        for r in self.network.origins:
            for s in r.getDests():
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
        
        self.milp.maximize(sum(self.milp.z[(r,s)] * r.getDemand(s) for r in self.network.origins for s in r.getDests()))
        
        self.milp.add_constraint(sum(self.milp.y[a] for a in self.network.candidates) <= self.network.B)
        
        self.milp.solve(log_output=False)
        y = {a:self.milp.y[a].solution_value for a in self.network.candidates}
        
        
        z = {(r,s): self.milp.z[(r,s)].solution_value for r in self.network.origins for s in r.getDests()}
        
        for a in y:
            a.y = y[a]
        
        print("check obj", self.calcObjZ(z))
        
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
                        
                        if r.id == 2 and s.id == 6:
                            print("\t", a, self.milp.x[(r,s)][a].solution_value, a.t_ff, msg)
                
        '''      
        
        obj = self.milp.objective_value
        
        self.milpy = y
        t_total = time.time() - t_total
        
        print(obj, t_total)
        print("validate", self.calcObj(y))
        return y, obj, t_total
        
    def compare(self):
        y_milp, obj_milp, t_milp = self.milp()
        y_bd, obj_bd, t_bd = self.benders()
        
        print("MILP", obj_milp, t_milp)
        #print("\t", y_milp)
        print("BD", obj_bd, t_bd)
        
               
    def initRMP(self):
        
        for a in self.network.links:
            a.y = 1
            
        for a in self.network.candidates:
            a.y = 0
            
            
        self.rmp = Model()
        
        self.rmp.zeta = {(r,s):self.rmp.continuous_var(lb=0, ub=1) for r in self.network.origins for s in r.getDests()}
        
        self.rmp.y = {a: self.rmp.integer_var(lb=0,ub=1) for a in self.network.candidates}
        
        self.rmp.add_constraint(sum(self.rmp.y[a] for a in self.network.candidates) <= self.network.B)
        
        self.rmp.maximize(sum(self.rmp.zeta[(r,s)] * r.getDemand(s) for r in self.network.origins for s in r.getDests()))
        
        
        
     
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
                
        t_total = time.time() - t_total
        
        
        print("validate", self.calcObj(besty))
        
        return besty, lb, t_total+t_init
     
    def solveRMP(self):
        self.rmp.solve(log_output=False)
        y = {a:self.rmp.y[a].solution_value for a in self.network.candidates}
        obj = self.rmp.objective_value
        
        #z = dict()
        z = {(r,s): self.rmp.zeta[(r,s)].solution_value for r in self.network.origins for s in r.getDests()}
        
        return y, obj, z
           
    def subproblem(self, y):
          
        for a in y:
            a.y = y[a]
            
        obj = 0
            
        for r in self.network.origins:
            for s in r.getDests():
                gamma_rs = 0
                
                self.network.dijkstras(r, self.max_cost, False)
                
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