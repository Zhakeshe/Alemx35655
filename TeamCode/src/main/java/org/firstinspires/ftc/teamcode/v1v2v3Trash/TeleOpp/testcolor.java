package org.firstinspires.ftc.teamcode.v1v2v3Trash.TeleOpp;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.ColorSensorConfig;

@Disabled
public class testcolor extends OpMode {
    ColorSensorConfig Color = new ColorSensorConfig();
    @Override
    public void init() {
        Color.init(hardwareMap);
    }
    @Override
    public void loop(){
        Color.getDetectedColor(telemetry);
    }
}
