package org.firstinspires.ftc.teamcode.v1v2v3Trash.auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class alys {

    //##################################
    private DcMotorEx Outtake1;
    private DcMotorEx Outtake2;

    private DcMotor Intake;

    private DcMotor Trans;
    //!!!!!!!!!!!!!!!!!!!!
    double integralSum1 = 0;

    public static double Kp1 = 0.01;
    public static double Ki1 = 0;
    public static double Kd1 = 0.0005;

    public static double Kf1 = 0.95;
    ElapsedTime timer1 = new ElapsedTime();
    private double lastError1 = 0;

    //##########################PID2
    double integralSum2 = 0;

    public static double Kp2 = 0.01;
    public static double Ki2 = 0;
    public static double Kd2 = 0.0005;

    public static double Kf2 = 0.95;
    ElapsedTime timer2 = new ElapsedTime();
    private double lastError2 = 0;

    public static double ref = 1700;
    public static double ref1 = 1800;

    //

    double integralSum3 = 0;

    public static double Kp3 = 0.01;
    public static double Ki3 = 0;
    public static double Kd3 = 0.0001;

    public static double Kf3 = 0.95;
    ElapsedTime timer3 = new ElapsedTime();
    private double lastError3 = 0;

    //

    double integralSum4 = 0;

    public static double Kp4 = 0.01;
    public static double Ki4 = 0;
    public static double Kd4 = 0.0001;

    public static double Kf4 = 0.95;
    ElapsedTime timer4 = new ElapsedTime();
    private double lastError4 = 0;


    private ElapsedTime stateTimer = new ElapsedTime();
    private ElapsedTime inTimer = new ElapsedTime();


    private enum FlywheelState{
        IDLE,
        SPIN_UP,
        RESET_GATE,
    }
    private enum IntakeState{
        InKos,
        Inson,
        Reset
    }

    private enum IntakeState1{
        InKos1,
        Inson1,
        Reset1
    }




    private FlywheelState flywheelState;

    private IntakeState intakeState;
    private IntakeState1 intakeState1;


    private double TransCloseTime = 1.3;
    private double TransCloseTime1 = 0.4;


    private double InOpenTime = 3;
    private double InOpenTime1 = 0.5;


    private int shotsRemaining = 0;
    private int intakeRemaining = 0;
    private int intakeRemaining1 = 0;


    private double ShootTime = 1.2;

    public void init(HardwareMap hwMap){
        Trans = hwMap.get(DcMotor.class, "Trans");

        Intake = hwMap.get(DcMotor.class, "Intake");

        Outtake1 = hwMap.get(DcMotorEx.class, "Outtake1");
        Outtake2 = hwMap.get(DcMotorEx.class, "Outtake2");

        Outtake1.setDirection(DcMotorSimple.Direction.REVERSE);
        Outtake2.setDirection(DcMotorSimple.Direction.FORWARD);

        Intake.setDirection(DcMotorSimple.Direction.REVERSE);
        Trans.setDirection(DcMotorSimple.Direction.REVERSE);

        Outtake1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Outtake2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Outtake1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Outtake2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelState = FlywheelState.IDLE;
        intakeState = IntakeState.InKos;
        intakeState1 = IntakeState1.InKos1;


    }
    public void update(){
        switch (flywheelState){
            case IDLE:
                if (shotsRemaining > 0){
                    Intake.setPower(1);
                    Trans.setPower(-1);
                    Outtake1.setVelocity(PIDcontrollLow1(ref1, Outtake1.getVelocity()));
                    Outtake2.setVelocity(PIDcontrollLow2(ref1, Outtake2.getVelocity()));
                    stateTimer.reset();

                    flywheelState = FlywheelState.SPIN_UP;
                }
                break;

            case SPIN_UP:
                if (stateTimer.seconds() > ShootTime){
                    Intake.setPower(1);
                    Trans.setPower(1);
                    Outtake1.setVelocity(PIDcontrollLow1(ref1, Outtake1.getVelocity()));
                    Outtake2.setVelocity(PIDcontrollLow2(ref1, Outtake2.getVelocity()));
                    shotsRemaining--;
                    stateTimer.reset();

                    flywheelState = FlywheelState.RESET_GATE;
                }
                break;
            case RESET_GATE:
                if (stateTimer.seconds() > TransCloseTime){
                    if (shotsRemaining > 0){
                        stateTimer.reset();
                        flywheelState = FlywheelState.SPIN_UP;
                    }

                    else {
                        Intake.setPower(0);
                        Trans.setPower(0);
                        Outtake1.setPower(0);
                        Outtake2.setPower(0);

                        flywheelState = FlywheelState.IDLE;
                    }
                }
                break;
        }

    }

    public void intake(){
        switch (intakeState){
            case InKos:
                if (intakeRemaining > 0){
                    Intake.setPower(1);
                    Trans.setPower(-1);

                    inTimer.reset();
                    intakeState = IntakeState.Inson;
                }
                break;

            case Inson:
                if (inTimer.seconds() > InOpenTime){
                    intakeRemaining--;
                    Intake.setPower(0);
                    Trans.setPower(0);

                    inTimer.reset();
                    intakeState = IntakeState.Reset;
                }
                break;

            case Reset:
                if (inTimer.seconds() > TransCloseTime1){
                    if (intakeRemaining > 0){
                        inTimer.reset();
                        intakeState = IntakeState.Inson;
                    }

                    else {
                        intakeState = IntakeState.InKos;
                    }
                }
                break;

        }
    }
    public void intake1(){
        switch (intakeState1){
            case InKos1:
                if (intakeRemaining1 > 0){
                    Intake.setPower(1);
                    Trans.setPower(-1);

                    stateTimer.reset();
                    intakeState1 = IntakeState1.Inson1;
                }
                break;

            case Inson1:
                if (stateTimer.seconds() > InOpenTime1){
                    intakeRemaining1--;
                    Intake.setPower(0);
                    Trans.setPower(0);

                    stateTimer.reset();
                    intakeState1 = IntakeState1.Reset1;
                }
                break;

            case Reset1:
                if (stateTimer.seconds() > TransCloseTime1){
                    if (intakeRemaining1 > 0){
                        stateTimer.reset();
                        intakeState1 = IntakeState1.Inson1;
                    }

                    else {
                        intakeState1 = IntakeState1.InKos1;
                    }
                }
                break;

        }
    }


    public void FireShots(int number0fShots){
        if (flywheelState == FlywheelState.IDLE){
            shotsRemaining = number0fShots;
        }
    }

    public void InShots(int numberOfIn){
        if (intakeState == IntakeState.InKos){
            intakeRemaining = numberOfIn;
        }
    }


    public void InShots1(int numberOfIn1){
        if (intakeState1 == IntakeState1.InKos1){
            intakeRemaining1 = numberOfIn1;
        }
    }


    public boolean isBusy(){
        return flywheelState != FlywheelState.IDLE;
    }

    public double PIDcontrollHigh1(double reference1, double state1){
        double error1 = reference1 - state1;
        integralSum1 += error1 * timer1.seconds();
        double derivative1 = (error1 - lastError1) / timer1.seconds();
        lastError1 = error1;

        timer1.reset();

        return (error1 * Kp1) + (derivative1 * Kd1) + (integralSum1 * Ki1) + (reference1 * Kf1);


    }
    public double PIDcontrollHigh2(double reference2, double state2){
        double error2 = reference2 - state2;
        integralSum2 += error2 * timer2.seconds();
        double derivative2 = (error2 - lastError2) / timer2.seconds();
        lastError2 = error2;

        timer2.reset();

        return (error2 * Kp2) + (derivative2 * Kd2) + (integralSum2 * Ki2) + (reference2 * Kf2);


    }

    public double PIDcontrollLow1(double reference3, double state3){
        double error3 = reference3 - state3;
        integralSum3 += error3 * timer3.seconds();
        double derivative3 = (error3 - lastError3) / timer3.seconds();
        lastError3 = error3;

        timer3.reset();

        return (error3 * Kp3) + (derivative3 * Kd3) + (integralSum3 * Ki3) + (reference3 * Kf3);


    }
    public double PIDcontrollLow2(double reference4, double state4){
        double error4 = reference4 - state4;
        integralSum4 += error4 * timer4.seconds();
        double derivative4 = (error4 - lastError4) / timer4.seconds();
        lastError4 = error4;

        timer4.reset();

        return (error4 * Kp4) + (derivative4 * Kd4) + (integralSum4 * Ki4) + (reference4 * Kf4);


    }

}
