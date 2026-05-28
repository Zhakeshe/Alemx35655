    package org.firstinspires.ftc.teamcode.configTamaraV3;

    import com.acmerobotics.dashboard.FtcDashboard;
    import com.acmerobotics.dashboard.config.Config;
    import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.DcMotorEx;
    import com.qualcomm.robotcore.hardware.DcMotorSimple;
    import com.qualcomm.robotcore.hardware.HardwareMap;
    import com.qualcomm.robotcore.hardware.Servo;
    import com.qualcomm.robotcore.util.ElapsedTime;

    import org.firstinspires.ftc.robotcore.external.Telemetry;

    @Config
    public class shooterAndIntake {

        private DcMotorEx shooter1, shooter2;
        private DcMotor intake, transfer;

        //private Servo blocker;

        boolean sht1 = false;
        boolean sht2 = false;
        boolean sht3 = false;

        boolean tt1 = false;
        boolean tt2 = false;


        //##########################PID1
        double integralSum1 = 0;

        public static double Kp1 = 0.006;
        public static double Ki1 = 0;
        public static double Kd1 = 0.00005;

        public static double Kf1 = 0.9;
        ElapsedTime timer1 = new ElapsedTime();
        private double lastError1 = 0;


        public static double ref = 2100;

        double targetRef = 0;

        //##########################PID2
        double integralSum2 = 0;

        public static double Kp2 = 0.001;
        public static double Ki2 = 0;
        public static double Kd2 = 0.001;

        public static double Kf2 = 0.9;
        ElapsedTime timer2 = new ElapsedTime();
        private double lastError2 = 0;

        

        public static double ref2 = 1450;



        TelemetryPacket packet = new TelemetryPacket();
        FtcDashboard dashboard = FtcDashboard.getInstance();


        public void init(HardwareMap hwMap){
            intake = hwMap.get(DcMotor.class, "intake");
            transfer = hwMap.get(DcMotor.class, "transfer");
            shooter1 = hwMap.get(DcMotorEx.class, "shooter1");
            shooter2 = hwMap.get(DcMotorEx.class, "shooter2");

            //blocker = hardwareMap.get(Servo.class, "blocker");

            intake.setDirection(DcMotorSimple.Direction.FORWARD);
            transfer.setDirection(DcMotorSimple.Direction.REVERSE);

            shooter1.setDirection(DcMotorSimple.Direction.FORWARD);
            shooter2.setDirection(DcMotorSimple.Direction.REVERSE);

            shooter1.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter1.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            shooter2.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            shooter2.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }

        public void shooterIntake(boolean grb, boolean glb, boolean g1a, Telemetry telemetry){

            packet.put("Target", 1800);
            packet.put("Velocity1", shooter1.getVelocity());
            packet.put("Velocity2", shooter2.getVelocity());
            telemetry.addData("v1: ", shooter1.getVelocity());

            dashboard.sendTelemetryPacket(packet);

            shooter1.setVelocity(targetRef);
            shooter2.setVelocity(targetRef);

            if (grb && !sht1){
                sht2 = true;
                if (intake.getPower() == 0){
                    sht1 = true;
                    sht2 = true;
                    sht3 = false;
                    intake.setPower(1);
                    transfer.setPower(-1);
                    //ref = 1300;
                    targetRef = PIDcontrollLow1((ref2), ((shooter1.getVelocity() + shooter2.getVelocity()) / 2));
                }else if(intake.getPower() > 0 && !sht3){
                    sht1 = true;
                    sht2 = true;
                    sht3 = true;
                    intake.setPower(1);
                    transfer.setPower(-1);
                    //ref = 1800;
                    targetRef = PIDcontrollHigh1((ref), ((shooter1.getVelocity() + shooter2.getVelocity()) / 2));
                }
                else if(sht3){
                    sht1 = true;
                    sht2 = true;
                    sht3 = false;
                    //intake.setPower(0);
                    transfer.setPower(0);
                    //ref = 0;
                    targetRef = 0;

                }
            } else if (glb) {
                sht1 = true;
                sht2 = true;
                sht3 = true;
                intake.setPower(0);
                transfer.setPower(0);
                //ref = 0;
                targetRef = 0;
            }else if (!grb & sht2) {
                sht1 = false;
            }

            if (g1a && !tt1){
                tt2 = false;
                if (transfer.getPower() < 0){
                    tt1 = true;
                    tt2 = true;
                    transfer.setPower(1);
                }else if(transfer.getPower() > 0){
                    tt1 = true;
                    tt2 = true;
                    transfer.setPower(-1);
                }
            } else if (!g1a && tt2) {
                tt1 = false;
            }

        }


        public double PIDcontrollHigh1(double reference1, double state1){
            double error1 = reference1 - state1;
            integralSum1 += error1 * timer1.seconds();
            double derivative1 = (error1 - lastError1) / timer1.seconds();
            lastError1 = error1;

            timer1.reset();

            return (error1 * Kp1) + (derivative1 * Kd1) + (integralSum1 * Ki1) + (reference1 * Kf1);


        }

        public double PIDcontrollLow1(double reference2, double state2){
            double error2 = reference2 - state2;
            integralSum2 += error2 * timer2.seconds();
            double derivative2 = (error2 - lastError2) / timer2.seconds();
            lastError2 = error2;

            timer2.reset();

            return (error2 * Kp2) + (derivative2 * Kd2) + (integralSum2 * Ki2) + (reference2 * Kf2);


        }
    }
