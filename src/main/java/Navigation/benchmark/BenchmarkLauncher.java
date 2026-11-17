package Navigation.benchmark;
import Navigation.Navigation3D.Normalization.ObjectiveWeights;
import Navigation.object.*;
import java.nio.file.Paths;

/** Shared scenario and vessel configuration for the paper experiments. */
public final class BenchmarkLauncher {
    static final class Scenario {
        final String id, name, dataFolder, navDate;
        final double startLat, startLon, endLat, endLon, depHour;
        final double latMin, latMax, lonMin, lonMax;
        final int nx, ny, startX, startY, endX, endY;

        Scenario(String id, String name, String dataFolder, String navDate,
                 double slat, double slon, double elat, double elon, double depH,
                 double latMin, double latMax, double lonMin, double lonMax,
                 int nx, int ny) {
            this.id = id; this.name = name; this.dataFolder = dataFolder;
            this.navDate = navDate; this.depHour = depH;
            this.startLat = slat; this.startLon = slon;
            this.endLat = elat; this.endLon = elon;
            this.latMin = latMin; this.latMax = latMax;
            this.lonMin = lonMin; this.lonMax = lonMax;
            this.nx = nx; this.ny = ny;
            // Grid start/end: 1-based → need to set in config
            // For now, let the grid start/end be auto-calculated from lat/lon mapping
            this.startX = (int)Math.round((slon - lonMin) / (lonMax - lonMin) * (nx - 1)) + 1;
            this.startY = (int)Math.round((slat - latMin) / (latMax - latMin) * (ny - 1)) + 1;
            this.endX   = (int)Math.round((elon - lonMin) / (lonMax - lonMin) * (nx - 1)) + 1;
            this.endY   = (int)Math.round((elat - latMin) / (latMax - latMin) * (ny - 1)) + 1;
        }
    }

    static final Scenario[] SCENARIOS = {
        // Six manuscript scenarios, numbered S1–S6.
        new Scenario("S1","TaiwanStrait_Cross","S1_S5_taiwan","20260311",
                28.000,122.500,24.500,119.500,0.0, 19,29,114,125, 132,120),
        new Scenario("S2","SouthChinaSea","S2_south_china_sea","20260313",
                15.500,114.000,9.000,110.500,12.0, 6,18,110,120, 121,145),
        new Scenario("S3","MiyakoStrait","S3_miyako_strait","20260311",
                26.500,124.500,25.000,127.000,6.0, 24,28,123,128, 61,49),
        new Scenario("S4","BashiChannel","S4_bashi_channel","20260312",
                24.000,122.000,18.500,124.000,18.0, 18,25,119,126, 85,85),
        new Scenario("S5","PenghuArchipelago","S1_S5_taiwan","20260312",
                25.500,122.000,22.000,120.000,12.0, 19,29,114,125, 132,120),

        new Scenario("S6","QingdaoToBusan_Korea","S6_qingdao_busan","20260311",
                36.000000,120.4166666667,35.0833333333,129.0833333333,0.0, 33,40,120,132, 145,85),

    };

    static final ObjectiveWeights COMPOSITE_W = new ObjectiveWeights(0.5, 0.3, 0.2);
    static NavigationConfig buildConfig(Scenario sc) {
        NavigationConfig cfg = new NavigationConfig();
        String inputRoot = System.getProperty("nav.inputRoot");
        boolean useUnifiedMissingPolicy = inputRoot != null && !inputRoot.isBlank();
        if (useUnifiedMissingPolicy
                && !"ocean_idw_v1".equals(System.getProperty("nav.missingPolicy"))) {
            throw new IllegalStateException(
                    "nav.inputRoot requires -Dnav.missingPolicy=ocean_idw_v1");
        }
        cfg.dataFolder = useUnifiedMissingPolicy
                ? Paths.get(inputRoot, sc.dataFolder).normalize().toString()
                : sc.dataFolder;
        cfg.navigationDate = sc.navDate;
        cfg.departureHour = sc.depHour;
        cfg.area = new NavigationArea(sc.latMin, sc.latMax, sc.lonMin, sc.lonMax);
        cfg.grid = new NavigationGrid(sc.nx, sc.ny, sc.startX, sc.startY, sc.endX, sc.endY);
        cfg.shipParameters = makeShip();
        cfg.obstacleFileName = "obstacle_map.json";
        return cfg;
    }

    static ShipParameters makeShip() {
        ShipParameters sp = new ShipParameters();
        if (Boolean.getBoolean("nav.r1.component")) {
            sp.shipDraft=6.15;sp.shipSpeed=9.256149419710122/0.5144;
            sp.shipLength=142.;sp.shipWidth=19.06;sp.shipDisplacement=8431.8;
            return sp;
        }
        sp.shipDraft = 4.0; sp.shipSpeed = 6.0; sp.shipLength = 60.0;
        sp.shipWidth = 13.0; sp.shipDisplacement = 1500.0;
        return sp;
    }

}
