import matplotlib.pyplot as plt
import csv



def main():
    x=[]
    y = []
    data = csv.reader(open("pts.log"))
    for l in data:
        a,b = map(float, l)
        x.append(a)
        y.append(b)

    plt.plot(x,y)
    plt.show()

if __name__ == "__main__":
    main()
