#!/usr/bin/env python3
from node_manager import ari_mover

if __name__ == "__main__":
	ari = ari_mover()
	while(1):
		ari.do()
		
