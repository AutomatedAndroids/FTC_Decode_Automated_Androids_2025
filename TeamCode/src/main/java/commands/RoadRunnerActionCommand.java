package commands;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.Action;
import com.arcrobotics.ftclib.command.CommandBase;

public class RoadRunnerActionCommand extends CommandBase {
    private final Action action;
    private final FtcDashboard dash = FtcDashboard.getInstance();
    private boolean finished = false;

    public RoadRunnerActionCommand(Action action) {
        this.action = action;
    }

    @Override
    public void execute() {
        TelemetryPacket packet = new TelemetryPacket();
        boolean running = action.run(packet);
        dash.sendTelemetryPacket(packet);
        finished = !running;
    }

    @Override
    public boolean isFinished() {
        return finished;
    }
}

