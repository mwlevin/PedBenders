#---modules
from src import Network
from src import Benders
from decimal import Decimal
#import polytope as pc
import sys

def main():



	
	net = 'Munich'
	
		
	num_baseline = 2000
	num_candidate = 40
	B = num_candidate/2
	max_cost = 2
	#max_cost = 20

	network = Network.Network(net, num_baseline, num_candidate, B)
	
	
	test = Benders.Benders(network, max_cost)

	
	#test.milp()
	test.compare()
	
if __name__=="__main__":
    main()