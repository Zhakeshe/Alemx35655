package org.firstinspires.ftc.teamcode.mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class ColorSensorConfig {
    NormalizedColorSensor colorSensor;

    public enum DetectedColor {
        GREEN,
        PURPLE,
        UNKNOWN
    }

    public void init(HardwareMap hwMap) {
        colorSensor = hwMap.get(NormalizedColorSensor.class, "ColorSensorGP");

    }

    public DetectedColor getDetectedColor(Telemetry telemetry) {
        NormalizedRGBA colors = colorSensor.getNormalizedColors();
        float normGreen, normPurple;
        normGreen = colors.green / colors.alpha;
        normPurple = colors.blue / colors.alpha;


        telemetry.addData("green", normGreen);
        telemetry.addData("purple", normPurple);

        return DetectedColor.UNKNOWN;
    }
}
