#!/usr/bin/env python


import fileinput
import sys


def main():
	inp = []
	string = []
	synolo = 0
	elastic = 0

	for line in fileinput.input():
		string = line
		string=string.replace("\n","")
		string=string.split(":")
		inp = inp+string
		value = int(string[3])
		synolo = value+synolo
		if value==50:
			elastic = elastic + 1
	
	if synolo>2000:
		score = -1.0
	else:
		score = 1.0
	print "score:",score
	k=1	
	if elastic==0:
		for i in inp[1::4]:
			
			print "set_limit:"+inp[k]+":cpu.shares:"+str(2000*int(inp[k+2])/synolo)
			k=k+4
	else:
		for i in inp[1::4]:
			
			if int(inp[k+2]) ==50 :
				print "set_limit:"+inp[k]+":cpu.shares:"+str((2000-synolo)/elastic+50)	
			else:
				print "set_limit:"+inp[k]+":cpu.shares:"+inp[k+2]
			k=k+4
if __name__ == "__main__":
	main()
			
