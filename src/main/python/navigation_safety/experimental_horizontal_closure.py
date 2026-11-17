"""Robust numerical solution of the unchanged experimental Gaussian closure.

Independent starting variances avoid preserving the wrong v/r amplitude ratio.
Pseudo-arclength follows folds when ordinary load continuation cannot. Neither
method changes coefficients or relaxes the original 1e-6 log-moment residual.
This module does not imply physical validation or production approval.
"""
import numpy as np
from scipy.optimize import least_squares

class HorizontalCovariance:
    def __init__(self,e,H,x0,U,n):
        self.e,self.H,self.x0,self.U,self.n=e,H,x0,U,n
        E=np.array([[0.,1/U,0.,.683/U],[0.,0.,0.,e.m.L/U]])
        self.hx=np.einsum('ik,nkj->nij',E,H[:,:,:3]);self.w0=x0@E.T
        self.scale=.5*e.h.RHO*U*U*e.m.L*e.m.L
        self.initial=self.pack(self.w0);self.evaluations=0
    def pack(self,w):
        cov=2*np.real(w.conj().T@w)/self.n**2
        sd=np.sqrt(np.diag(cov))
        if not np.isfinite(sd).all() or np.any(sd<=0):raise ArithmeticError('Invalid response covariance')
        rho=cov[0,1]/np.prod(sd)
        return np.r_[np.log(sd),np.arctanh(np.clip(rho,-1+1e-12,1-1e-12))]
    def coefficients(self,z):
        sv,sr=np.exp(z[:2]);vr=np.tanh(z[2])*sv*sr
        def d(c):
            return np.array([2*c['v_abs_v']*np.sqrt(2/np.pi)*sv+2*c['v2r']*vr+c['vr2']*sr**2,
                             2*c['r_abs_r']*np.sqrt(2/np.pi)*sr+c['v2r']*sv**2+2*c['vr2']*vr])
        dy=self.scale*d(self.e.CFG['Y']);dn=self.scale*self.e.m.L*d(self.e.CFG['N'])+.683*dy
        return np.vstack([np.zeros(2),dy,dn])
    def solve(self,z,load=1.):
        A=self.coefficients(z)*load;loop=np.eye(2)[None]-self.hx@A
        det=loop[:,0,0]*loop[:,1,1]-loop[:,0,1]*loop[:,1,0]
        w=np.column_stack([(loop[:,1,1]*self.w0[:,0]-loop[:,0,1]*self.w0[:,1])/det,
                          (-loop[:,1,0]*self.w0[:,0]+loop[:,0,0]*self.w0[:,1])/det])
        return w,A
    def residual(self,z,load=1.):
        self.evaluations+=1
        return self.pack(self.solve(z,load)[0])-z
    def fit(self,start):
        fit=least_squares(self.residual,start,max_nfev=140,xtol=1e-11,ftol=1e-11,gtol=1e-11)
        error=float(np.max(abs(self.residual(fit.x))))
        return fit.x,error,fit.nfev
    def arclength(self):
        fun=lambda y:self.residual(y[:3],y[3])
        y=np.r_[self.initial,0.];tangent=np.array([0.,0.,0.,1.]);step=.04;history=[]
        for k in range(400):
            # Centred differences on just four unknowns, independent of the
            # number of samples. Full original frequency grid is retained.
            delta=1e-5
            J=np.column_stack([(fun(y+np.eye(4)[i]*delta)-fun(y-np.eye(4)[i]*delta))/(2*delta) for i in range(4)])
            t=np.linalg.svd(J,full_matrices=True)[2][-1]
            if np.dot(t,tangent)<0:t=-t
            tangent=t;prediction=y+step*t
            fit=least_squares(lambda z:np.r_[fun(z),np.dot(z-prediction,t)],prediction,
                              max_nfev=80,xtol=1e-10,ftol=1e-10,gtol=1e-10)
            error=float(np.max(abs(fun(fit.x))))
            if error>1e-7:
                step/=2
                if step<1e-5:break
                continue
            old=y.copy();y=fit.x;step=min(.06,step*1.1)
            history.append(dict(load=float(y[3]),residual=error,tangent_load=float(t[-1])))
            if (old[3]-1)*(y[3]-1)<=0:
                z,error,_=self.fit(y[:3])
                if error<=1e-6:return z,error,history
            if y[3]<-.01 or y[3]>2 or np.max(abs(y[:3]))>30:break
        raise ArithmeticError('Covariance continuation failed without an admissible endpoint')
    def robust(self):
        attempts=[]
        # Fixed numerical starts, identical for every input. No old response,
        # measurement, scene, route or successful neighbouring state is used.
        starts=[(0,0,None),(-2,-2,None),(-1,-1,None),(1,1,None),(2,2,None),
                (-1,0,None),(0,-1,None),(1,0,None),(0,1,None),
                (-2,1,None),(1,-2,None),(-1,1,0),(1,-1,0),(1,1,0)]
        for dv,dr,corr in starts:
            initial=self.initial.copy();initial[:2]+=[dv,dr]
            if corr is not None:initial[2]=corr
            z,error,nfev=self.fit(initial)
            attempts.append(dict(log_variance_start=[dv,dr],correlation_parameter_start=corr,residual=error,nfev=nfev))
            if error<=1e-6:history=[];break
        else:z,error,history=self.arclength()
        w,A=self.solve(z)
        x=self.x0+np.einsum('nki,ni->nk',self.H[:,:,:3],w@A.T)
        return x,dict(hull_closure_relative_log_residual=error,hull_closure_evaluations=self.evaluations,
            hull_closure_attempts=attempts,covariance_parameters=z.tolist(),equivalent_hull_coefficients=A.tolist(),
            arclength_steps=len(history),arclength_history=history,
            numerical_policy='independent_fixed_variance_starts_then_pseudo_arclength_v1')
