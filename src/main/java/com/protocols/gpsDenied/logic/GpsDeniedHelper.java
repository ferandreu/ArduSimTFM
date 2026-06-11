package com.protocols.gpsDenied.logic;

import com.api.API;
import com.api.ArduSim;
import com.api.ArduSimTools;
import com.api.FileTools;
import com.api.ProtocolHelper;
import com.api.communications.WirelessModel;
import com.protocols.gpsDenied.pojo.PositionSample;
import com.setup.Param;
import es.upv.grc.mapper.DrawableCircleGeo;
import es.upv.grc.mapper.GUIMapPanelNotReadyException;
import es.upv.grc.mapper.Mapper;

import java.awt.BasicStroke;
import java.awt.Color;
import com.api.pojo.location.Waypoint;
import com.api.copter.TakeOffListener;
import com.api.swarm.formations.Formation;
import com.api.swarm.formations.FormationFactory;
import com.protocols.gpsDenied.gui.GpsDeniedDialogApp;
import com.protocols.gpsDenied.gui.GpsDeniedSimProperties;
import com.setup.Text;
import com.setup.sim.logic.SimParam;
import com.uavController.UAVParam;
import es.upv.grc.mapper.Location2DGeo;
import es.upv.grc.mapper.Location2DUTM;
import es.upv.grc.mapper.Location3D;
import es.upv.grc.mapper.Location3DUTM;
import es.upv.grc.mapper.LocationNotReadyException;
import io.dronefleet.mavlink.common.MavCmd;
import io.dronefleet.mavlink.common.MavFrame;
import io.dronefleet.mavlink.util.EnumValue;
import javafx.application.Platform;
import javafx.stage.Stage;
import org.javatuples.Pair;

import javax.swing.*;
import java.io.File;
import java.io.FileInputStream;
import java.io.IOException;
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;
import java.util.Properties;
import java.util.PropertyResourceBundle;
import java.util.ResourceBundle;
import java.util.concurrent.atomic.AtomicInteger;

public class GpsDeniedHelper extends ProtocolHelper {

    private static final EnumValue<MavCmd> WP_CMD      = EnumValue.of(MavCmd.MAV_CMD_NAV_WAYPOINT);
    private static final EnumValue<MavCmd> TAKEOFF_CMD = EnumValue.of(MavCmd.MAV_CMD_NAV_TAKEOFF);
    private static final EnumValue<MavCmd> LAND_CMD    = EnumValue.of(MavCmd.MAV_CMD_NAV_LAND);

    @Override
    public void setProtocol() {
        this.protocolString = GpsDeniedParam.PROTOCOL_TEXT;
    }

    @Override
    public boolean loadMission() { return true; }

    @Override
    public JDialog openConfigurationDialog() { return null; }

    @Override
    public void openConfigurationDialogFX() {
        Platform.runLater(() -> new GpsDeniedDialogApp().start(new Stage()));
    }

    @Override
    public void configurationCLI() {
        GpsDeniedSimProperties properties = new GpsDeniedSimProperties();
        ResourceBundle resources;
        try {
            FileInputStream fis = new FileInputStream(SimParam.protocolParamFile);
            resources = new PropertyResourceBundle(fis);
            fis.close();
            Properties p = new Properties();
            for (String key : resources.keySet()) {
                p.setProperty(key, resources.getString(key));
            }
            properties.storeParameters(p, resources);
        } catch (IOException e) {
            e.printStackTrace();
            ArduSimTools.warnGlobal(Text.LOADING_ERROR, Text.PROTOCOL_PARAMETERS_FILE_NOT_FOUND);
            System.exit(0);
        }
    }

    @Override
    public void initializeDataStructures() {
        int numUAVs = API.getArduSim().getNumUAVs();
        GpsDeniedParam.state         = new AtomicInteger[numUAVs];
        GpsDeniedParam.flyingTargets = new Location3DUTM[numUAVs];
        for (int i = 0; i < numUAVs; i++) {
            GpsDeniedParam.state[i] = new AtomicInteger(0);
        }
        GpsDeniedParam.leaderLanded.set(false);
        GpsDeniedParam.estimatedLeaderPosition.set(null);
        GpsDeniedParam.lastValidEstimateTimeMs.set(0L);
        GpsDeniedParam.observerLastHeard        = new long[numUAVs];
        GpsDeniedParam.observerLastPos          = new Location2DUTM[numUAVs];
        @SuppressWarnings("unchecked")
        ArrayDeque<Long>[] ts = new ArrayDeque[numUAVs];
        for (int i = 0; i < numUAVs; i++) ts[i] = new ArrayDeque<>();
        GpsDeniedParam.observerPacketTimestamps = ts;
        GpsDeniedParam.leaderPositionLog    = Collections.synchronizedList(new ArrayList<>());
        GpsDeniedParam.observerRangeCircles = new DrawableCircleGeo[numUAVs];
        GpsDeniedParam.intermediateTarget   = null;
        GpsDeniedParam.leaderStartUTM       = null;

        Formation linear = FormationFactory.newFormation(Formation.Layout.LINEAR);
        linear.init(numUAVs, 10.0);
        UAVParam.groundFormation.set(linear);
    }

    @Override
    public String setInitialState() { return GpsDeniedText.START; }

    @Override
    public Pair<Location2DGeo, Double>[] setStartingLocation() {
        int numUAVs = API.getArduSim().getNumUAVs();
        @SuppressWarnings("unchecked")
        Pair<Location2DGeo, Double>[] startingLocation = new Pair[numUAVs];

        Location3D centerRef = new Location3D(
                GpsDeniedParam.centerLatitude,
                GpsDeniedParam.centerLongitude,
                0.0);
        Location3DUTM centerUTM = centerRef.getUTMLocation3D();
        GpsDeniedParam.leaderStartUTM = new Location2DUTM(centerUTM.x, centerUTM.y);

        double yaw = GpsDeniedParam.centerYaw;
        double X   = GpsDeniedParam.leaderFlightDistance;
        double Y   = GpsDeniedParam.observerSideDistance;
        double alt = GpsDeniedParam.altitude;

        double dxAlong =  Math.sin(yaw);
        double dyAlong =  Math.cos(yaw);
        double dxRight =  Math.cos(yaw);
        double dyRight = -Math.sin(yaw);

        // Leader (UAV 0): final target and optional intermediate waypoint
        GpsDeniedParam.flyingTargets[0] = new Location3DUTM(
                centerUTM.x + dxAlong * X,
                centerUTM.y + dyAlong * X,
                alt);
        if (GpsDeniedParam.midpointAlong > 0) {
            GpsDeniedParam.intermediateTarget = new Location3DUTM(
                    centerUTM.x + dxAlong * GpsDeniedParam.midpointAlong + dxRight * GpsDeniedParam.midpointRight,
                    centerUTM.y + dyAlong * GpsDeniedParam.midpointAlong + dyRight * GpsDeniedParam.midpointRight,
                    alt);
        } else {
            GpsDeniedParam.intermediateTarget = null;
        }
        startingLocation[0] = Pair.with(centerRef.getGeoLocation(), yaw);

        // Observers (UAV 1…N-1): evenly spaced along the route, alternating sides.
        // First observer at the start of the route, last at the end.
        // Even index → right (+Y), odd index → left (-Y).
        int numObservers = numUAVs - 1;
        if (numObservers > 0) {
            for (int i = 0; i < numObservers; i++) {
                double along = (numObservers == 1)
                        ? X * 0.5
                        : (double) i / (numObservers - 1) * X;
                double side = (i % 2 == 0) ? +1.0 : -1.0;
                double fx = centerUTM.x + dxAlong * along + side * dxRight * Y;
                double fy = centerUTM.y + dyAlong * along + side * dyRight * Y;
                GpsDeniedParam.flyingTargets[i + 1] = new Location3DUTM(fx, fy, alt);
                try {
                    startingLocation[i + 1] = Pair.with(
                            new Location3DUTM(fx, fy, 0.0).getGeo(), yaw);
                } catch (LocationNotReadyException e) {
                    API.getGUI(0).exit("GpsDenied: geo conversion failed for observer " + i + ": " + e.getMessage());
                    return null;
                }
            }
        }

        // Build and register missions for trajectory display + SITL upload
        @SuppressWarnings("unchecked")
        List<Waypoint>[] missions = new ArrayList[numUAVs];
        try {
            double startLat = GpsDeniedParam.centerLatitude;
            double startLon = GpsDeniedParam.centerLongitude;
            Location2DGeo leaderEnd = GpsDeniedParam.flyingTargets[0].getGeo();
            if (GpsDeniedParam.intermediateTarget != null) {
                Location2DGeo midGeo = GpsDeniedParam.intermediateTarget.getGeo();
                missions[0] = buildLeaderMissionWithMidpoint(
                        startLat, startLon,
                        midGeo.latitude, midGeo.longitude,
                        leaderEnd.latitude, leaderEnd.longitude, alt);
            } else {
                missions[0] = buildMission(startLat, startLon,
                                           leaderEnd.latitude, leaderEnd.longitude, alt);
            }
            for (int i = 0; i < numObservers; i++) {
                Location2DGeo obsGeo = GpsDeniedParam.flyingTargets[i + 1].getGeo();
                missions[i + 1] = buildMission(obsGeo.latitude, obsGeo.longitude,
                                               obsGeo.latitude, obsGeo.longitude, alt);
            }
        } catch (LocationNotReadyException e) {
            API.getGUI(0).exit("GpsDenied: mission geo conversion failed: " + e.getMessage());
            return null;
        }
        API.getCopter(0).getMissionHelper().setMissionsLoaded(missions);

        return startingLocation;
    }

    /** Leader mission with an intermediate waypoint shown on the map between takeoff and final target. */
    private List<Waypoint> buildLeaderMissionWithMidpoint(
            double sLat, double sLon,
            double midLat, double midLon,
            double eLat, double eLon, double alt) {
        List<Waypoint> m = new ArrayList<>();
        m.add(new Waypoint(0, true,  MavFrame.MAV_FRAME_GLOBAL_RELATIVE_ALT, WP_CMD,      0,0,0,0, sLat,   sLon,   0,   1));
        m.add(new Waypoint(1, false, MavFrame.MAV_FRAME_GLOBAL_RELATIVE_ALT, TAKEOFF_CMD, 0,0,0,0, sLat,   sLon,   alt, 1));
        m.add(new Waypoint(2, false, MavFrame.MAV_FRAME_GLOBAL,               WP_CMD,      0,0,0,0, midLat, midLon, alt, 1));
        m.add(new Waypoint(3, false, MavFrame.MAV_FRAME_GLOBAL,               WP_CMD,      0,0,0,0, eLat,   eLon,   alt, 1));
        m.add(new Waypoint(4, false, MavFrame.MAV_FRAME_GLOBAL,               LAND_CMD,    0,0,0,0, eLat,   eLon,   0,   0));
        return m;
    }

    private List<Waypoint> buildMission(double sLat, double sLon,
                                         double eLat, double eLon, double alt) {
        List<Waypoint> m = new ArrayList<>();
        m.add(new Waypoint(0, true,  MavFrame.MAV_FRAME_GLOBAL_RELATIVE_ALT, WP_CMD,      0,0,0,0, sLat, sLon, 0,   1));
        m.add(new Waypoint(1, false, MavFrame.MAV_FRAME_GLOBAL_RELATIVE_ALT, TAKEOFF_CMD, 0,0,0,0, sLat, sLon, alt, 1));
        m.add(new Waypoint(2, false, MavFrame.MAV_FRAME_GLOBAL,               WP_CMD,      0,0,0,0, eLat, eLon, alt, 1));
        m.add(new Waypoint(3, false, MavFrame.MAV_FRAME_GLOBAL,               LAND_CMD,    0,0,0,0, eLat, eLon, 0,   0));
        return m;
    }

    @Override
    public boolean sendInitialConfiguration(int numUAV) { return true; }

    @Override
    public void startThreads() {
        int numUAVs = API.getArduSim().getNumUAVs();
        // Leader listener: receives observer broadcasts and estimates position.
        new GpsDeniedLeaderListenerThread().start();
        for (int i = 0; i < numUAVs; i++) {
            new GpsDeniedDroneThread(i).start();
            if (i > 0) {
                // Observer talker: broadcasts own position to the leader.
                new GpsDeniedTalkerThread(i).start();
            }
        }
    }

    /**
     * Takes off ALL UAVs simultaneously in GUIDED mode and blocks until every
     * one is airborne.  This is the confirmed-working takeoff pattern and puts
     * every drone in Guided_armed state before the experiment starts.
     */
    @Override
    public void setupActionPerformed() {
        int numUAVs = API.getArduSim().getNumUAVs();
        List<Thread> ts = new ArrayList<>(numUAVs);
        for (int i = 0; i < numUAVs; i++) {
            final int n = i;
            Thread t = API.getCopter(n).takeOff(GpsDeniedParam.altitude, new TakeOffListener() {
                @Override public void onCompleteActionPerformed() {}
                @Override public void onFailure() {}
            });
            ts.add(t);
            t.start();
        }
        for (Thread t : ts) {
            try { t.join(); } catch (InterruptedException ignored) {
                Thread.currentThread().interrupt();
            }
        }
    }

    /** Drone threads handle the rest once isExperimentInProgress() is true. */
    @Override
    public void startExperimentActionPerformed() {
        if (Param.role != ArduSim.SIMULATOR_GUI) return;
        int numUAVs = API.getArduSim().getNumUAVs();
        BasicStroke stroke = new BasicStroke(1.5f);
        Color color = new Color(0, 100, 220);
        double R = (Param.selectedWirelessModel == WirelessModel.FIXED_RANGE)
                ? Param.fixedRange : GpsDeniedParam.MAX_RANGE_5GHZ_M;
        for (int i = 1; i < numUAVs; i++) {
            try {
                Location2DGeo center = GpsDeniedParam.flyingTargets[i].getGeo();
                GpsDeniedParam.observerRangeCircles[i] =
                        Mapper.Drawables.addCircleGeo(2, center, R, color, stroke);
            } catch (LocationNotReadyException | GUIMapPanelNotReadyException e) {
                API.getGUI(0).log("GpsDenied: could not draw range circle for observer " + i + ": " + e.getMessage());
            }
        }
    }

    /**
     * Called periodically by ArduSim during the experiment.
     * Drone threads manage the landing sequence themselves:
     * leader lands after reaching target, then signals observers to land.
     * Calling land() here would terminate the flight prematurely.
     */
    @Override
    public void forceExperimentEnd() {}

    @Override
    public String getExperimentResults() {
        List<PositionSample> log = GpsDeniedParam.leaderPositionLog;
        if (log == null || log.isEmpty()) return "No position samples recorded.\n";

        double sumErr = 0, maxErr = 0, minErr = Double.MAX_VALUE;
        int estimatedCount = 0, inAreaCount = 0;
        for (PositionSample s : log) {
            if (!s.hasEstimate()) continue;
            estimatedCount++;
            sumErr += s.error2D;
            if (s.error2D > maxErr) maxErr = s.error2D;
            if (s.error2D < minErr) minErr = s.error2D;
        }

        String errStr = (estimatedCount == 0)
                ? "  No samples with estimate.\n"
                : String.format("  Mean 2-D error   : %.2f m%n", sumErr / estimatedCount)
                + String.format("  Max  2-D error   : %.2f m%n", maxErr)
                + String.format("  Min  2-D error   : %.2f m%n", minErr);

        return "Leader position estimation:\n"
             + "  Total samples    : " + log.size() + "\n"
             + "  With estimate    : " + estimatedCount + "\n"
             + "  In intersection  : " + inAreaCount + " (" +
               String.format("%.1f%%", 100.0 * inAreaCount / log.size()) + ")\n"
             + errStr;
    }

    @Override
    public String getExperimentConfiguration() {
        return "Protocol:               " + GpsDeniedParam.PROTOCOL_TEXT             + "\n"
             + "Altitude:               " + GpsDeniedParam.altitude                  + " m\n"
             + "Leader speed:           " + GpsDeniedParam.leaderSpeed               + " m/s\n"
             + "Leader flight dist (X): " + GpsDeniedParam.leaderFlightDistance      + " m\n"
             + "Observer side dist (Y): " + GpsDeniedParam.observerSideDistance      + " m\n"
             + "Heading (yaw):          " + Math.toDegrees(GpsDeniedParam.centerYaw) + " deg\n";
    }

    @Override
    public void logData(String folder, String baseFileName, long baseNanoTime) {
        if (Param.role == ArduSim.SIMULATOR_GUI && GpsDeniedParam.observerRangeCircles != null) {
            for (DrawableCircleGeo circle : GpsDeniedParam.observerRangeCircles) {
                if (circle != null) {
                    try {
                        Mapper.Drawables.removeDrawable(circle);
                    } catch (GUIMapPanelNotReadyException e) {
                        // Map already closed — no action needed.
                    }
                }
            }
        }

        List<PositionSample> log = GpsDeniedParam.leaderPositionLog;
        if (log == null || log.isEmpty()) return;

        ArduSim arduSim = API.getArduSim();
        long expStartMs = arduSim.getExperimentStartTime();

        StringBuilder sb = new StringBuilder(log.size() * 120 + 150);
        sb.append("time_s,est_x,est_y,true_x,true_y,error_2d_m,observers_in_range\n");

        for (PositionSample s : log) {
            double t = (s.timeMs - expStartMs) / 1000.0;
            String estX  = s.hasEstimate() ? String.format(Locale.US, "%.3f", s.estX)   : "";
            String estY  = s.hasEstimate() ? String.format(Locale.US, "%.3f", s.estY)   : "";
            String err   = s.hasEstimate() ? String.format(Locale.US, "%.3f", s.error2D): "";
            sb.append(String.format(Locale.US, "%.3f,%s,%s,%.3f,%.3f,%s,%d%n",
                    t, estX, estY, s.trueX, s.trueY, err, s.observersInRange));
        }

        FileTools ft = API.getFileTools();
        File csvFile = new File(folder + File.separator + baseFileName + "_gpsDenied_positions.csv");
        ft.storeFile(csvFile, sb.toString());
    }

    @Override
    public void openPCCompanionDialog(JFrame PCCompanionFrame) {}
}
