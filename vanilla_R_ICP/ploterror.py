  import numpy as np
  import matplotlib.pyplot as plot
  import sys

  plot.figure(1)

  file = open('sample.txt','r')
  x=[]
  m=1
  y=[]
  for line in file:
      trainingSet = line.split(' ')
      for value in trainingSet:
	x.append(value)
	y.append(m)
	m=m+1
plot.plot(y,x)
      
