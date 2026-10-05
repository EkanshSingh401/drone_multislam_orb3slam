import sys, json, numpy as np
from scipy.spatial.transform import Rotation as Rot
R_true = np.array([[0,-1,0],[0,0,-1],[1,0,0]],float)
p_true = {0:np.array([0.0424,0.01174,-0.00552]),1:np.array([-0.0526,0.01174,-0.00552])}
K_true = np.array([446.802773,446.802773,424.0,240.0])
for d in sys.argv[1:]:
    run=d.split('_')[0][-6:]
    A=np.loadtxt(d+'/ov_state_est.txt'); S=np.loadtxt(d+'/ov_state_std.txt')
    w=json.load(open(d+'/../'+d.split('/')[-1]+'_win.json')) if False else None
    t=A[:,0]; n=len(t); sl=slice(n//2,None)   # second half: settled
    print(f"== {run}  dt_cam_imu: final {A[-1,17]*1e3:+.2f} ms, 2nd-half median {np.median(A[sl,17])*1e3:+.2f} ms, std(sigma) final {S[-1,17]*1e3:.2f} ms" if S.shape[1]>17 else '')
    for c,o in ((0,19),(1,34)):
        k=A[:,o:o+4]; dist=A[:,o+4:o+8]; q=A[:,o+8:o+12]; p=A[:,o+12:o+15]
        # JPL q (xyzw) -> R = Hamilton(q)^T
        Rj=Rot.from_quat(q).as_matrix().transpose(0,2,1)
        ang=[np.degrees(np.linalg.norm(Rot.from_matrix(R_true.T@r).as_rotvec())) for r in Rj]
        ang2=[np.degrees(np.linalg.norm(Rot.from_matrix(R_true.T@r.T).as_rotvec())) for r in Rj]
        if np.median(ang2)<np.median(ang): ang=ang2
        ang=np.array(ang)
        dk=np.median(k[sl],0)-K_true
        print(f" cam{c} fx,fy,cx,cy - SDF: {np.round(dk,2)} px  (final {np.round(k[-1]-K_true,2)})")
        print(f"      dist median {np.round(np.median(dist[sl],0),4)}")
        print(f"      rot err median {np.median(ang[sl]):.3f} deg, final {ang[-1]:.3f}; trans err median {np.round((np.median(p[sl],0)-p_true[c])*1e3,1)} mm")
