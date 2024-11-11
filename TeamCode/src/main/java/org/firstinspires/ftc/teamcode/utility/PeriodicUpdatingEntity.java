package org.firstinspires.ftc.teamcode.utility;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;

public interface PeriodicUpdatingEntity {

    void update(TelemetryPacket packet);
}
