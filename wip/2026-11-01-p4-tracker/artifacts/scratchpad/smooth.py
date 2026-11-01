import sys, numpy as np
f, dev = sys.argv[1], sys.argv[2]
t=[];X=[]
for l in open(f):
    p=l.split()
    if p[1]==dev and p[2]=="POSE": t.append(float(p[0])); X.append([float(x) for x in p[3:6]])
t=np.array(t); X=np.array(X); res=[]
for i in range(len(t)):
    m=np.abs(t-t[i])<0.06
    if m.sum()<7: continue
    tt=t[m]-t[i]; res.append([X[i,k]-np.polyfit(tt,X[m,k],2)[-1] for k in range(3)])
r=np.linalg.norm(np.array(res),axis=1)*1e3
print("n %d jitter rms %.2f mm p99 %.1f max %.1f"%(len(r),np.sqrt(np.mean(r**2)),np.percentile(r,99),r.max()))
