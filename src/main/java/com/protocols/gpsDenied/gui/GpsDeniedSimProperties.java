package com.protocols.gpsDenied.gui;

import com.api.ArduSimTools;
import com.protocols.gpsDenied.logic.GpsDeniedParam;
import com.setup.Text;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Iterator;
import java.util.Map;
import java.util.Properties;
import java.util.ResourceBundle;

/**
 * Stores protocol parameters loaded from the GUI or the .properties file.
 *
 * IMPORTANT (ArduSim convention): field names here MUST match the fx:id names
 * in the FXML, the key names in the .properties file, and the variable names
 * read in the controller — the framework uses reflection to map them.
 */
public class GpsDeniedSimProperties {

    // ── Parameters exposed to the GUI / .properties file ────────────────
    /** Starting latitude of the leader (degrees). */
    public double latitude             = GpsDeniedParam.centerLatitude;
    /** Starting longitude of the leader (degrees). */
    public double longitude            = GpsDeniedParam.centerLongitude;
    /** Heading the leader flies (degrees in the GUI; converted to radians on store). */
    public double yaw                  = 0.0;
    /** (X) Straight-line distance the leader travels (metres). */
    public double leaderFlightDistance = GpsDeniedParam.leaderFlightDistance;
    /** (Y) Perpendicular distance of observer drones from the leader's path (metres). */
    public double observerSideDistance = GpsDeniedParam.observerSideDistance;
    /** Cruise altitude for all UAVs (metres, relative). */
    public double altitude             = GpsDeniedParam.altitude;
    /** Leader cruise speed (m/s). */
    public double leaderSpeed          = GpsDeniedParam.leaderSpeed;
    /** Distance along heading to the optional intermediate waypoint (m). 0 = disabled. */
    public double midpointAlong        = GpsDeniedParam.midpointAlong;
    /** Perpendicular offset of the intermediate waypoint (m, positive = right). */
    public double midpointRight        = GpsDeniedParam.midpointRight;

    // ────────────────────────────────────────────────────────────────────

    public boolean storeParameters(Properties guiParams, ResourceBundle fileParams) {
        // Merge: GUI values override file values when present
        Properties merged = new Properties();
        for (String key : fileParams.keySet()) {
            merged.setProperty(key,
                    guiParams.containsKey(key)
                            ? guiParams.getProperty(key)
                            : fileParams.getString(key));
        }

        // Map field names → Field objects via reflection (ArduSim convention)
        Field[] fields = this.getClass().getFields();
        Map<String, Field> fieldMap = new HashMap<>();
        for (Field f : fields) { fieldMap.put(f.getName(), f); }

        Iterator<Object> itr = merged.keySet().iterator();
        while (itr.hasNext()) {
            String key   = itr.next().toString();
            String value = merged.getProperty(key);
            if (!fieldMap.containsKey(key)) { continue; }
            Field f = fieldMap.get(key);
            try {
                String type = f.getType().toString();
                if (type.equals("double")) {
                    f.setDouble(this, Double.parseDouble(value));
                } else {
                    ArduSimTools.warnGlobal(Text.LOADING_ERROR, Text.ERROR_STORE_PARAMETERS + type);
                    return false;
                }
            } catch (IllegalAccessException e) {
                return false;
            }
        }

        String error = validate();
        if (!error.isEmpty()) {
            ArduSimTools.warnGlobal(Text.LOADING_ERROR, "GpsDenied: invalid parameter: " + error);
            return false;
        }
        applyToParam();
        return true;
    }

    private String validate() {
        if (altitude             <= 0) return "altitude";
        if (leaderSpeed          <= 0) return "leaderSpeed";
        if (leaderFlightDistance <= 0) return "leaderFlightDistance";
        if (observerSideDistance <= 0) return "observerSideDistance";
        if (midpointAlong < 0)         return "midpointAlong (must be >= 0; use 0 to disable)";
        if (midpointAlong > 0 && midpointAlong >= leaderFlightDistance)
            return "midpointAlong must be less than leaderFlightDistance";
        return "";
    }

    private void applyToParam() {
        GpsDeniedParam.centerLatitude       = latitude;
        GpsDeniedParam.centerLongitude      = longitude;
        GpsDeniedParam.centerYaw            = yaw * Math.PI / 180.0;  // degrees → radians
        GpsDeniedParam.altitude             = altitude;
        GpsDeniedParam.leaderSpeed          = leaderSpeed;
        GpsDeniedParam.leaderFlightDistance = leaderFlightDistance;
        GpsDeniedParam.observerSideDistance = observerSideDistance;
        GpsDeniedParam.midpointAlong        = midpointAlong;
        GpsDeniedParam.midpointRight        = midpointRight;
    }
}
