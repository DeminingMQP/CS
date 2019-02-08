#!flask/bin/python

import sys

from flask import Flask, render_template, request, redirect, Response
import random, json
import math
from graphics import *
from rosmain import *
import threading
from random import *

global Grid
Grid = []
global myVars
myVars = []
global startLoc
startLoc = []
global startGridLoc
startGridLoc = []

def shutdown_server():
    func = request.environ.get('werkzeug.server.shutdown')
    if func is None:
        raise RuntimeError('Not running with the Werkzeug Server')
    func()

def determineChargeLoc(minX,minY):
	dist = -1
	for i in range(0,len(Grid[0])):
		for j in range(0,len(Grid)):
			if (dist<0 or ((((minX+j-startLoc[0])**2+(minY+i-startLoc[1])**2)**(1/2))<dist) and Grid[j][i]!=0):
				dist = (((minX+j-startLoc[0])**2+(minY+i-startLoc[1])**2)**(1/2))
				locI = i
				locJ = j
	startGridLoc.append(locI)
	startGridLoc.append(locJ)
	Grid[locJ][locI] = 7

def printGraphicGrid(Grid):
	win = GraphWin("Minefield", 600, 600)
	graphicGrid = []
	for i in range(0, len(Grid)):
		graphicGrid.append([])
		for j in range(0, len(Grid[0])):
			graphicGrid[i].append(
				Rectangle(Point(50 + 10 * j - 5, 500 - 10 * i - 5), Point(50 + 10 * j + 5, 500 - 10 * i + 5)))
			graphicGrid[i][j].draw(win)
	for i in range(len(Grid[0]) - 1, -1, -1):
		for j in range(0, len(Grid)):
			if Grid[j][i] == 5 or Grid[j][i] == 2:
				graphicGrid[j][i].setFill('black')
			if Grid[j][i] == 0:
				graphicGrid[j][i].setFill('white')
			if Grid[j][i] == 3:
				graphicGrid[j][i].setFill('red')
			if Grid[j][i] == 7:
				graphicGrid[j][i].setFill('gray')
	win.getMouse()

def determineSlope(x1,x2,y1,y2):
    if(x2-x1 == 0):
        print("vertical")
        return "NaN"
    print(y2,y1,x2,x1,(y2-y1)/(x2-x1))
    return (y2-y1)/(x2-x1)

def determineIntercept(x,y,m):
    if(m=="NaN"):
        return "NaN"
    return y-(x*m)

def printGrid(Grid):
	for j in range(len(Grid) - 1, -1, -1):
		for i in range(0, len(Grid[0])):
			print(str(Grid[j][i])+" ",end="")
		print( i,j,"\n")

def myFunction(form):
	x = []
	y = []
	print(int(len(form)/2))
	for index in range(0, int(len(form)/2)):

		print("(" + form["lat" + str(index)] + "," + form["lng" + str(index)] + ")")
		x.append(float(form["lat" + str(index)]))
		y.append(float(form["lng" + str(index)]))
		print("HEre")
		print(x)

	print(x)
	print(y)
	m = []
	b = []
	coorX =[]
	coorY =[]
	maxX = -400000000
	minX = 400000000
	maxY = -400000000
	minY = 400000000
	for index in range(0, len(x)):
		print(x[index])
		x[index] = math.floor(x[index] * 50000)
		y[index] = math.floor(y[index] * 50000)
	startLoc.append(x[0])
	startLoc.append(y[0])
	for index in range(1, len(x)):
		if (x[index] < minX):
			minX = x[index]
		if (x[index] > maxX):
			maxX = x[index]
		if (y[index] < minY):
			minY = y[index]
		if (y[index] > maxY):
			maxY = y[index]
		if (index == len(x) - 1):
			print("here")
			m.append(determineSlope(x[index], x[1], y[index], y[1]))
			b.append(determineIntercept(x[index], y[index], m[index-1]))
			coorX.append((x[index],x[1]))
			coorY.append((y[index], y[1]))
		else:
			print("here2")
			m.append(determineSlope(x[index], x[index + 1], y[index], y[index + 1]))
			print(x,y,m,"here3")
			b.append(determineIntercept(x[index], y[index], m[index-1]))
			coorX.append((x[index], x[index+1]))
			coorY.append((y[index], y[index+1]))
	print(m)
	print(b)
	print(maxX)
	print(minX)
	print(maxY)
	print(minY)
	for index in range(0, len(x)-1):
		print("y = " + str(m[index]) + "x + " + str(b[index]))
	xAxis = int((maxX - minX))
	yAxis = int((maxY - minY))
	print(xAxis, yAxis)
	for index in range (0,xAxis):
		Grid.append([])
		for index2 in range (0,yAxis):
			Grid[index].append(0)
	print(len(Grid),len(Grid[0]),"POLK")
	#Grid = [[1] * yAxis for _ in range(xAxis)]
	for i in range(len(Grid[0]) - 1, -1, -1):
		for j in range(0, len(Grid)):
			intersects = 0
			for k in range(0, len(x)-1):
				print(m[k], minX + j, b[k], minY + i)
				#TODO add case for slope 0
				if m[k] == "NaN":
					if (x[k+1] < minX + j) and minY+i>min(coorY[k][0],coorY[k][1]) and minY+i<max(coorY[k][0],coorY[k][1]):
						intersects += 1
						print(k, "heree", 1)
						print(x[k+1], minX + j)
				elif m[k] > 0:
					print("Negative")
					if ((m[k] * (minX + j) + b[k]) > (minY + i)) and minY+i>=min(coorY[k][0],coorY[k][1]) and minY+i<max(coorY[k][0],coorY[k][1]):
					#if ((m[k] * (minX + j) + b[k]) > (minY + i)):
						intersects += 1
						print(k, "heree", 2)
					else:
						print(i, j)
						print("failed", ((m[k] * (minX + j) + b[k]) > (minY + i)),
							  minY + i >= min(coorY[k][0], coorY[k][1]), minY + i < max(coorY[k][0], coorY[k][1]))

				elif m[k] < 0:
					print("positive")
					if ((m[k] * (minX + j) + b[k]) < (minY + i)) and minY+i>=min(coorY[k][0],coorY[k][1]) and minY+i<max(coorY[k][0],coorY[k][1]):
					#if ((m[k] * (minX + j) + b[k]) < (minY + i)):
						intersects += 1
						print(k, "heree", 3)
					else:
						print(i,j)
						print("failed", ((m[k] * (minX + j) + b[k]) < (minY + i)),minY+i>=min(coorY[k][0],coorY[k][1]),minY+i<max(coorY[k][0],coorY[k][1]))
				# if(abs((m[k] * (minX + j) + b[k]) - (minY + i)))<1:
				# 	intersects +=1
				if m[k]=="NaN":
					print("vertical")
				else:
					print(m[k] * (minX + j) + b[k], minY + i, "line" + str(k), m[k], (minX + j), b[k],
					  min(coorY[k][0], coorY[k][1]), max(coorY[k][0], coorY[k][1]))
			print("intersects", intersects)


			if intersects % 2 == 1:
				print(i,j)
				Grid[j][i] = 5
			#for k in range(0, len(x) - 1):
				#if math.floor((m[k] * (minX + j) + b[k])) == (minY + i):
					#Grid[j][i] = 4
			print("", )
			if i==1 and j==0:
				Grid[j][i]=3
	#printGrid(Grid)
	print(len(Grid))
	myVars.append(x)
	myVars.append(y)
	myVars.append(minX)
	myVars.append(minY)
	determineChargeLoc(minX, minY)
	print(startGridLoc[0], startGridLoc[1], "startLoc", maxX-minX,maxY-minY)
	shutdown_server()


app = Flask(__name__)

@app.route('/')
def output():
    #return "Hello World"
	# serve index template
	return render_template('GoogleMapSelect.html', name='Joe')

@app.route('/receiver', methods = ['POST'])
def worker():
	# read json + reply
	print(request.form)
	result = " "
	form = request.form
	myFunction(form)
	return result

def mainFunc():
	print("FUNC")

def myPolygon():
	win = GraphWin("Minefield", 600, 600)
	for i in range(0,len(myVars[0])-1):
		myLine = Line(Point(myVars[0][i]-myVars[2],myVars[1][i]-myVars[3]),Point(myVars[0][i+1]-myVars[2],myVars[1][i+1]-myVars[3]))
		myLine.setOutline("red")
		myLine.draw(win)
	for i in range (0,len(Grid)):
		for j in range(0,len(Grid[0])):
			if Grid[i][j] == 1:
				myPoint = Point(i,j)
				myPoint.draw(win)
	win.getMouse()

def addMines():
	masterGrid = []
	for i in range (0,len(Grid)):
		masterGrid.append([])
		for j in range(0,len(Grid[0])):
			masterGrid[i].append(Grid[i][j])
			x = 0
			if masterGrid[i][j] == 5:
				if abs(startGridLoc[0]-j)>1 and abs(startGridLoc[1]-i)>1:
					x = randint(0,30)
					if x == 10:
						masterGrid[i][j] = 3
					else:
						masterGrid[i][j] = 2
				else:
					masterGrid[i][j] = 2
	return masterGrid




if __name__ == '__main__':
	# run!
	print("working")
	app.run()
	masterGrid = addMines()
	#printGraphicGrid(Grid)
	#printGraphicGrid(masterGrid)
	
	print(startGridLoc[1],startGridLoc[0], "startloc")
	navigate(masterGrid,Grid,startGridLoc[1],startGridLoc[0])
	#myPolygon()

