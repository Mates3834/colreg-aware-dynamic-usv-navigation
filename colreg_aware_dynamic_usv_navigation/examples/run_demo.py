import matplotlib.pyplot as plt
from src.simulation import run

grid,path,goal,h,m=run()
print(m)

plt.figure()
plt.imshow(grid.occ,cmap="gray_r",origin="lower")
plt.plot(path[:,0],path[:,1],"--",label="Global A* path")
plt.plot(h["own"][:,0],h["own"][:,1],label="Own USV")

for j in range(h["targets"].shape[1]):
    plt.plot(h["targets"][:,j,0],h["targets"][:,j,1],label=f"Target {j+1}")

plt.scatter([goal[0]],[goal[1]],marker="*",s=120,label="Goal")
plt.xlabel("x")
plt.ylabel("y")
plt.title("COLREG-Aware Dynamic USV Navigation")
plt.grid(True)
plt.legend()
plt.show()
