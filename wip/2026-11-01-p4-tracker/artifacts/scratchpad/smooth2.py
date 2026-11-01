import sys, numpy as np
f, dev = sys.argv[1], sys.argv[2]
t=[];X=[]
for l in open(f):
    p=l.split()
    if len(p)>5 and p[1]==dev and p[2]=="POSE": t.append(float(p[0])); X.append([float(x) for x in p[3:6]])
t=np.array(t); X=np.array(X)
o=np.argsort(t,kind="stable"); t=t[o]; X=X[o]
res=[]; spd=[]
for i in range(0,len(t)):
    m=np.abs(t-t[i])<0.06
    if m.sum()<7: continue
    tt=t[m]-t[i]; c=[np.polyfit(tt,X[m,k],2) for k in range(3)]
    res.append([X[i,k]-c[k][-1] for k in range(3)]); spd.append(np.linalg.norm([c[k][-2] for k in range(3)]))
r=np.linalg.norm(np.array(res),axis=1)*1e3; spd=np.array(spd)
mv=spd>0.05
print("%s %s: n %d rate %.0f/s | moving (n %d): jitter rms %.2f mm p99 %.1f max %.1f | still: rms %.3f mm"%(f.split("/")[-1],dev,len(t),len(t)/(t[-1]-t[0]),mv.sum(),np.sqrt(np.mean(r[mv]**2)),np.percentile(r[mv],99),r[mv].max(),np.sqrt(np.mean(r[~mv]**2))))
