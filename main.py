#---modules
from src import Network
from src import Benders
from decimal import Decimal
#import polytope as pc
import sys

def main():



	
	net = 'Munich'
	
		
	num_baseline = 400
	num_candidate = 100
	B = num_candidate/4
	
	
	random_seed = 10
	
	max_cost = 0
	
	if net == 'Sioux Falls':
		max_cost = 10
	elif net == 'Anaheim':
		max_cost = 2*5280
	elif net == 'Munich':
		max_cost = 2






	network = Network.Network(net, num_baseline, num_candidate, B, random_seed)
	
	test = Benders.Benders(network, max_cost)

	
	#test.milp()
	test.compare()
	
if __name__=="__main__":
    main()