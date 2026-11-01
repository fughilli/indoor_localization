import json,sys,bisect,math
def poses(f):
    P=[json.loads(l) for l in open(f) if '"type":"pose"' in l]
    P.sort(key=lambda x:x["t"]); return P
def cmp(fa,fb):
    a=poses(fa); b=poses(fb); ta=[x["t"] for x in a]
    d=[];vec=[0,0,0];n=0
    for x in b:
        i=bisect.bisect_left(ta,x["t"])
        if i==0 or i>=len(a): continue
        p,q=a[i-1],a[i]
        if q["t"]-p["t"]>0.02: continue
        w=(x["t"]-p["t"])/(q["t"]-p["t"])
        ip=[p["p"][k]+w*(q["p"][k]-p["p"][k]) for k in range(3)]
        dv=[x["p"][k]-ip[k] for k in range(3)]
        d.append(math.hypot(*dv)); vec=[vec[k]+dv[k] for k in range(3)]; n+=1
    d.sort();q=lambda v,f:v[int(f*(len(v)-1))]
    print("%s vs %s: n %d |dp| median %.2f p90 %.2f p99 %.2f mm; mean dp (%.2f, %.2f, %.2f) mm"%(fb.split('/')[-1],fa.split('/')[-1],n,q(d,.5)*1e3,q(d,.9)*1e3,q(d,.99)*1e3,*[v/n*1e3 for v in vec]))
cmp(sys.argv[1],sys.argv[2])
