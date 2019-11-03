import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('tailleBD2MSteiner.txt')
i = 0
for column in data.T:
  if i == 1:
  	plt.plot(data[:,0], column, label = "Base de Test 2 avec méthode 2")
  if i == 2 : 
  	plt.plot(data[:,2],column,label = "Base de Test 1 avec méthode 2")
  i = i + 1

plt.xlabel("Nombre de point du graphe")
plt.ylabel("Taille de EDC (en nombre de points)")
plt.legend()
plt.show()

