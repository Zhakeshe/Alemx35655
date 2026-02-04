package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebCam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

@Disabled
public class CAM extends OpMode {
    private DcMotor motorTest;

    AprilTagWebCam aprilTagWebCam = new AprilTagWebCam();

    @Override
    public void init() {
        motorTest = hardwareMap.get(DcMotor.class, "motorTest");

        aprilTagWebCam.init(hardwareMap, telemetry);
    }

    @Override
    public void loop() {
        aprilTagWebCam.update();
        AprilTagDetection tag = aprilTagWebCam.getTagSpecificId(11);
        aprilTagWebCam.displayDetectionTelemetry(tag);

        if (tag != null){
            double y = tag.ftcPose.y;
            motorTest.setPower(y / 100);
            
        }
        telemetry.update();
    }
}
