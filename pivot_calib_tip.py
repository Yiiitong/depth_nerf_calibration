import numpy as np

mat = np.loadtxt('save_points_np_calib.txt')
mat_nr = int(mat.shape[0]/4)

R_list = []
t_list = []
T_list = []
for i in range(mat_nr):
	R = mat[4*i:4*i+3,0:3]
	t = mat[4*i:4*i+3,3]
	T = mat[4*i:4*i+4]

	R_list.append(R)
	t_list.append(t)
	T_list.append(T)

R_sub = []
t_sub = []
for i in range(mat_nr):
	index_sub = (i+1)%mat_nr
	R_sub.append( R_list[i]-R_list[index_sub] )
	t_sub.append( t_list[i]-t_list[index_sub] )

Rn = np.concatenate(R_sub,axis=0)
tn = np.concatenate(t_sub,axis=0) 
tn= -tn.reshape((-1,1))
pinv = np.linalg.pinv(Rn)
t = pinv.dot(tn)

tip_T = np.identity(4)
tip_T[:3,3] = t.T
# ~ print(t)
# ~ print(t.shape)
result_list = []
for T_i in T_list:
	result_list.append(np.matmul(T_i,tip_T)[:3,3]*1000)
	
print(np.stack(result_list,0))
print("var :",np.var(np.stack(result_list,0),axis=0))

print("calib :",tip_T[:3,3])
	

