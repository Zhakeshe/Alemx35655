package org.firstinspires.ftc.teamcode.pedroPathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.Encoder;
import com.pedropathing.ftc.localization.constants.TwoWheelConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .forwardZeroPowerAcceleration(-53.5177703)
            .lateralZeroPowerAcceleration(-33.6262034)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.01, 0, 0.01  ,0.03))
            .headingPIDFCoefficients(new PIDFCoefficients(1, 0, 0.02, 0.01))
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.5,0.0,0.0001,0.6,0.025))
            .mass(15);

    public static PathConstraints pathConstraints = new PathConstraints(0.99, 100, 1.8, 1);

    public static MecanumConstants driveConstants = new MecanumConstants()
            .xVelocity(51.065506)
            .yVelocity(27.569887)
            .maxPower(1)
            .rightFrontMotorName("rfex2")
            .rightRearMotorName("rrex3")
            .leftRearMotorName("lrex1")
            .leftFrontMotorName("lfex0")
            .leftFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.REVERSE);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .twoWheelLocalizer(localizerConstants)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .build();
    }



    public static TwoWheelConstants localizerConstants = new TwoWheelConstants()
            .forwardTicksToInches(0.0020494488232558404)
            .strafeTicksToInches(-0.0010905)
            .forwardPodY(-1.3779)
            .strafePodX(-7.4803)
            .forwardEncoderDirection(Encoder.FORWARD)
            .strafeEncoderDirection(Encoder.FORWARD)
            .forwardEncoder_HardwareMapName("Outtake1")
            .strafeEncoder_HardwareMapName("Outtake2")
            .IMU_HardwareMapName("imu")
            .IMU_Orientation(
                    new RevHubOrientationOnRobot(
                            RevHubOrientationOnRobot.LogoFacingDirection.UP,
                            RevHubOrientationOnRobot.UsbFacingDirection.LEFT
                    )
            );
}