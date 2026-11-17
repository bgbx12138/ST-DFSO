import java.io.*;
/** Same frozen nonlinear RK4 arithmetic as the Python reference; no fitting. */
public strictfp final class FrozenRollBridge {
 public static void main(String[] args)throws Exception {
  DataInputStream in=new DataInputStream(new BufferedInputStream(System.in,1<<20));
  DataOutputStream out=new DataOutputStream(new BufferedOutputStream(System.out,1<<20));
  while(true){
   int n;try{n=in.readInt();}catch(EOFException end){break;}if(n<0)break;
   double dt=in.readDouble(),I=in.readDouble(),C=in.readDouble(),alpha=in.readDouble(),beta=in.readDouble();
   double[] f=new double[n];for(int j=0;j<n;j++)f[j]=in.readDouble();
   double[] x=new double[n];double xx=0,vv=0,inv=1/I;
   for(int j=0;j<n-1;j++){
    double f0=f[j],f1=f[j+1],fm=.5*(f0+f1);
    double k1x=vv,k1v=(f0-C*xx-alpha*vv-beta*Math.abs(vv)*vv)*inv;
    double a=xx+.5*dt*k1x,b=vv+.5*dt*k1v,k2x=b,k2v=(fm-C*a-alpha*b-beta*Math.abs(b)*b)*inv;
    a=xx+.5*dt*k2x;b=vv+.5*dt*k2v;double k3x=b,k3v=(fm-C*a-alpha*b-beta*Math.abs(b)*b)*inv;
    a=xx+dt*k3x;b=vv+dt*k3v;double k4x=b,k4v=(f1-C*a-alpha*b-beta*Math.abs(b)*b)*inv;
    xx+=dt/6*(k1x+2*k2x+2*k3x+k4x);vv+=dt/6*(k1v+2*k2v+2*k3v+k4v);x[j+1]=xx;
   }
   for(double v:x)out.writeDouble(v);out.flush();
  }
 }
}
