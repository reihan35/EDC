import numpy as np
import matplotlib.pyplot as plt

data = np.loadtxt('temps.txt')
i = 0
for column in data.T:
  if i == 1:
  	plt.plot(data[:,0], column, label = "Base de Test 1 avec méthode 1")
  if i == 2 : 
  	plt.plot(data[:,2],column,label = "Base de Test 1 avec méthode 2")
  i = i + 1

plt.xlabel("Nombre de point du graphe")
plt.ylabel("Temps d'exécution en milliseconde")
plt.legend()
plt.show()

