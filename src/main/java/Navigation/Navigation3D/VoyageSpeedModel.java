package Navigation.Navigation3D;

import Navigation.object.ShipParameters;
import com.fasterxml.jackson.databind.JsonNode;
import com.fasterxml.jackson.databind.ObjectMapper;
import java.nio.file.Paths;

/** Source-based, approximate steady direct-wind/propulsion balance. See docs/VOYAGE_SPEED.md. */
public final class VoyageSpeedModel {
    private VoyageSpeedModel() {}
    public static final String MODEL = "BLENDERMANN_FIXED_RPM_5415_APPROX_V1";
    private static final double RHO=1025.0, NU=1.19e-6;
    private static final JsonNode P=parameters();
    private static final double AIR=p("air_density_kg_m3"), AF=p("frontal_area_m2"), AL=p("lateral_area_m2"),
        CT=p("wind_transverse_coefficient"), CH=p("wind_head_coefficient"), CA=p("wind_aft_coefficient"),
        DELTA=p("wind_cross_parameter"), D=p("propeller_diameter_m"), N=p("propeller_count");
    private static final double A=P.get("thrust_polynomial").get(0).asDouble(),
        B=P.get("thrust_polynomial").get(1).asDouble(), C=P.get("thrust_polynomial").get(2).asDouble();
    private static final double JZERO=(-B-Math.sqrt(B*B-4*A*C))/(2*C);
    private static JsonNode parameters() {
        try {
            JsonNode n=new ObjectMapper().readTree(Paths.get(System.getProperty("nav.speedConfig","config/voyage_speed.json")).toFile());
            if(!MODEL.equals(n.path("model").asText()) || n.path("wake_fraction").asDouble(1)!=0 || n.path("thrust_deduction").asDouble(1)!=0)
                throw new IllegalArgumentException("UNSUPPORTED_SPEED_MODEL_CONFIG");
            return n;
        } catch(Exception e){throw new ExceptionInInitializerError(e);}
    }
    private static double p(String key){double v=P.path(key).asDouble(Double.NaN);if(!Double.isFinite(v)||v<=0)throw new IllegalArgumentException(key);return v;}

    /** The existing resistance-work proxy, evaluated with speed through water.
     * Reynolds number uses relative-water speed, never the angle of the current. */
    public static double waterResistance(ShipParameters ship,double u) {
        if(!Double.isFinite(u)||u<0)return Double.NaN;
        double l=ship.shipLength/.97, b=ship.shipWidth, draft=ship.shipDraft;
        double cb=ship.shipDisplacement*1000/(RHO*ship.shipLength*b*draft);
        double area=1.7*l*draft+cb*l*b;
        double re=Math.max(1e5,u*l/NU), cf=.075/Math.pow(Math.log10(re)-2,2)+.0004;
        double cp=20.0/draft<=1.5?.27:.17;
        return .5*RHO*(cf*area+cp*b*draft)*u*u;
    }
    public static double energyJ(ShipParameters ship,double u,double hours) {
        return hours>=0?waterResistance(ship,u)*u*hours*3600:Double.NaN;
    }
    /** Positive means resistance. Apparent-air vector is expressed relative to the ground velocity. */
    public static double airResistance(double headingE,double headingN,double groundE,double groundN,double windE,double windN) {
        double ae=groundE-windE, an=groundN-windN, speed=Math.hypot(ae,an);
        if(speed==0)return 0;
        double cos=Math.max(-1,Math.min(1,(ae*headingE+an*headingN)/speed));
        double cl=(cos>=0?CH:CA)*AF/AL;
        double sin2=4*cos*cos*(1-cos*cos);
        return .5*AIR*AL*speed*speed*cl*cos/(1-.5*DELTA*(1-cl/CT)*sin2);
    }
    public static double nominalRps(ShipParameters ship) {
        double u=ship.shipSpeed*.5144, v=u/D;
        double resistance=waterResistance(ship,u)+airResistance(0,1,0,u,0,0);
        double right=resistance/(N*RHO*Math.pow(D,4));
        return (-B*v+Math.sqrt(B*B*v*v-4*A*(C*v*v-right)))/(2*A);
    }
    public static double thrust(double u,double rps){double j=u/(rps*D);return N*RHO*rps*rps*Math.pow(D,4)*(A+B*j+C*j*j);}
    public static final class State {
        public final double waterSpeed,groundSpeed,headingDeg,rps,balanceResidualN;
        State(double u,double vg,double h,double n,double residual){waterSpeed=u;groundSpeed=vg;headingDeg=h;rps=n;balanceResidualN=residual;}
        public boolean valid(){return Double.isFinite(waterSpeed)&&waterSpeed>0&&Double.isFinite(groundSpeed)&&groundSpeed>0;}
    }
    private static State invalid(){return new State(Double.NaN,Double.NaN,Double.NaN,Double.NaN,Double.NaN);}
    private static double residual(ShipParameters ship,double u,double n,double e,double north,double along,double cross,double ce,double cn,double we,double wn) {
        double vg=Math.sqrt(Math.max(0,u*u-cross*cross))+along;
        double he=(vg*e-ce)/u, hn=(vg*north-cn)/u;
        return thrust(u,n)-waterResistance(ship,u)-airResistance(he,hn,vg*e,vg*north,we,wn);
    }
    /** Ground track is constrained; the water heading compensates for cross-current.
     * An unbracketed balance or impossible forward track is unavailable, never a clipped artificial speed. */
    public static State solve(ShipParameters ship,double bearing,double we,double wn,double ce,double cn) {
        for(double x:new double[]{bearing,we,wn,ce,cn,ship.shipSpeed})if(!Double.isFinite(x))return invalid();
        if(ship.shipSpeed<=0)return invalid();
        double angle=Math.toRadians(bearing),e=Math.sin(angle),north=Math.cos(angle);
        double along=ce*e+cn*north,cross=ce*north-cn*e,n=nominalRps(ship);
        double low=Math.max(1e-7,Math.hypot(cross,Math.min(0,along))+1e-8),high=n*D*JZERO;
        if(low>=high)return invalid();
        double flo=residual(ship,low,n,e,north,along,cross,ce,cn,we,wn),fhi=residual(ship,high,n,e,north,along,cross,ce,cn,we,wn);
        if(!Double.isFinite(flo+fhi)||flo<=0||fhi>=0)return invalid();
        for(int k=0;k<36;k++) {
            double mid=(low+high)/2, f=residual(ship,mid,n,e,north,along,cross,ce,cn,we,wn);
            if(f>0)low=mid;else high=mid;
        }
        double u=(low+high)/2, vg=Math.sqrt(u*u-cross*cross)+along;
        // Same declared domain for TIME, ENERGY and motion objectives.
        double nominal=9.256149419710122;
        if(u<.95*nominal||u>1.05*nominal)return invalid();
        double h=Math.toDegrees(Math.atan2(vg*e-ce,vg*north-cn));if(h<0)h+=360;
        return new State(u,vg,h,n,residual(ship,u,n,e,north,along,cross,ce,cn,we,wn));
    }
}
