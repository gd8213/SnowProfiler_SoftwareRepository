# -*- coding: utf-8 -*-
"""
Created on Thu Dec  2 22:33:06 2021

@author: Felbermayr Simon
"""

fileObject = open("data.txt", "r")
data = fileObject.read()
print(data)





###############################################################
##############  other option #################################
###############################################################

# f = open("data.txt", "r")

# while(True):
# 	#read next line
# 	line = f.readline()
# 	#if line is empty, you are done with all lines in the file
# 	if not line:
# 		break
# 	#you can access the line
# 	print(line.strip())

# #close file
# f.close