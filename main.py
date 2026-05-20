#---modules
from src import Network
from src import Benders
from decimal import Decimal
#import polytope as pc
import sys

def main():



	
	net = 'SiouxFalls'
	
	
	num_baseline = 100
	num_candidate = 400
	
	
	
	
	
	
	
	random_seed = 10
	
	max_cost = 0
	
	if net == 'SiouxFalls':
		max_cost = 10
		num_baseline = 0
		num_candidate = 20
	elif net == 'Anaheim':
		max_cost = 2*5280
	elif net == 'Munich':
		max_cost = 2
		num_baseline = 100
		num_candidate = 400


	B = num_candidate*0.5



	network = Network.Network(net, num_baseline, num_candidate, B, max_cost, random_seed)
	
	test = Benders.Benders(network, max_cost)

	
	#test.milp()
	test.compare()
	
if __name__=="__main__":
    main()