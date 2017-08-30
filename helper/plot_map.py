

import csv
import matplotlib.pyplot as plt


data = csv.reader(open("../data/highway_map.csv"), delimiter=' ')

x = []
y = []
for l in data:
    a,b,k,j,n = map(float, l)
    x.append(a)
    y.append(b)

plt.scatter(x,y)
plt.show()
