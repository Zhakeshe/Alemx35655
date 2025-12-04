package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.mechanisms.ColorSensorConfig;

@TeleOp
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
