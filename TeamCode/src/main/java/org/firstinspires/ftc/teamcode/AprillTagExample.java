package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.mechanisms.AprilTagWebCam;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.opencv.dnn.DetectionModel;

@Autonomous
public class AprillTagExample extends OpMode {
    AprilTagWebCam aprilTagWebCam = new AprilTagWebCam();

    private Servo qwe;

    double servoPos = 0.75;

    final double kP = 0.01;



    @Override
    public void init() {
        aprilTagWebCam.init(hardwareMap, telemetry);
        qwe = hardwareMap.get(Servo.class, "qwe");
        qwe.setPosition(servoPos);
        qwe.setDirection(Servo.Direction.FORWARD);

    }

    @Override
    public void loop() {
        aprilTagWebCam.update();
        AprilTagDetection tag = aprilTagWebCam.getTagSpecificId(11);
        aprilTagWebCam.displayDetectionTelemetry(tag);


        if (tag != null){
            double y = tag.ftcPose.y;
            qwe.setPosition((y * 0.01) - 0.5);
            telemetry.addData("Y = ", y);
            telemetry.addData("Servo", (y * 0.01) - 0.5);
        }



        telemetry.update();


    }

}
