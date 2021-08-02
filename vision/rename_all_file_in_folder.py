
import os
import re
import sys
def renameall():
	fileList = os.listdir(r"/home/lai/Desktop/custom_deep_learning/JPEGImages")# file path
	print("before :"+str(fileList))		#print out all the file name
	currentpath = os.getcwd()		#get the current path
	os.chdir(r"/home/lai/Desktop/custom_deep_learning/JPEGImages")# change the current path to the target folder path
	num=1		
	for fileName in fileList:		# read out all the file in the folder
		pat=".+\.(jpg|png|gif)"		# match the file name with order
		pattern = re.findall(pat,fileName)		# start to match with the file name
		os.rename(fileName,(str(num)+'.'+pattern[0]))		# rename the file
		num = num+1		#
	print("---------------------------------------------------")
	os.chdir(currentpath)		# change back to the directory
	sys.stdin.flush()		# refresh
	print("after :"+str(os.listdir(r"/home/lai/Desktop/custom_deep_learning/JPEGImages")))#print out the file inside the folder

renameall()


