package com.protocols.gpsDenied.logic;

import com.api.API;
import com.api.ArduSim;
import com.api.GUI;
import com.api.communications.lowLevel.LowLevelCommLink;
import com.api.copter.Copter;
import com.esotericsoftware.kryo.io.Output;
import com.protocols.gpsDenied.pojo.GpsDeniedMessage;
import es.upv.grc.mapper.Location2DUTM;

import java.util.Arrays;

/**
 * One instance per observer drone (numUAV >= 1).
 * Broadcasts this drone's current UTM position at a fixed period so the
 * leader can determine which observers are within communication range and
 * use that to estimate its own approximate position.
 */
class GpsDeniedTalkerThread extends Thread {

    private final int numUAV;
    private final ArduSim arduSim;
    private final GUI gui;
    private final Copter copter;
    private final LowLevelCommLink link;
    private final byte[] outBuffer;
    private final Output output;

    GpsDeniedTalkerThread(int numUAV) {
        super("GpsDenied-Talker-" + numUAV);
        this.numUAV    = numUAV;
        this.arduSim   = API.getArduSim();
        this.gui       = API.getGUI(numUAV);
        this.copter    = API.getCopter(numUAV);
        this.link      = LowLevelCommLink.getCommLink(numUAV);
        this.outBuffer = new byte[LowLevelCommLink.DATAGRAM_MAX_LENGTH];
        this.output    = new Output(outBuffer);
    }

    @Override
    public void run() {
        while (!arduSim.isExperimentInProgress()) {
            arduSim.sleep(GpsDeniedParam.STATE_CHANGE_TIMEOUT);
        }
        gui.logVerboseUAV("GpsDenied observer " + numUAV + ": starting broadcast.");
        while (!GpsDeniedParam.leaderLanded.get()) {
            sendPosition();
            arduSim.sleep(GpsDeniedParam.BROADCAST_PERIOD_MS);
        }
        gui.logVerboseUAV("GpsDenied observer " + numUAV + ": broadcast stopped.");
    }

    private void sendPosition() {
        Location2DUTM loc = copter.getLocationUTM();

        output.reset();
        output.writeShort(GpsDeniedMessage.OBSERVER_POSITION);
        output.writeInt(numUAV);
        output.writeDouble(loc.x);
        output.writeDouble(loc.y);
        output.writeLong(System.currentTimeMillis());   // sent timestamp for delay estimation
        output.flush();
        link.sendBroadcastMessage(Arrays.copyOf(outBuffer, output.position()));
    }
}
