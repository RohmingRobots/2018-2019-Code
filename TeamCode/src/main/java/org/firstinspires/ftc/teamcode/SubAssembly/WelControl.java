package org.firstinspires.ftc.teamcode.SubAssembly;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubAssembly.Claimer.ClaimerControl;
import org.firstinspires.ftc.teamcode.SubAssembly.DriveTrain.DriveControl;
import org.firstinspires.ftc.teamcode.SubAssembly.Leds.LedControl;
import org.firstinspires.ftc.teamcode.SubAssembly.Lift.LiftControl;
import org.firstinspires.ftc.teamcode.SubAssembly.Miner.MinerControl;
import org.firstinspires.ftc.teamcode.SubAssembly.Sensors.ColorControl;
import org.firstinspires.ftc.teamcode.SubAssembly.Sensors.IMUcontrol;
import org.firstinspires.ftc.teamcode.SubAssembly.Sensors.TofControl;
import org.firstinspires.ftc.teamcode.SubAssembly.Vucam.VucamControl;
import org.firstinspires.ftc.teamcode.Utilities.UserControl;

public class WelControl {
    /* Declare private class object */
    private LinearOpMode opmode = null;     /* local copy of opmode class */
    private ElapsedTime runtime = new ElapsedTime();

    //initializing motors
    private DcMotor FrontRightM = null;
    private DcMotor FrontLeftM = null;
    private DcMotor BackRightM = null;
    private DcMotor BackLeftM = null;

    /* Declare public class object */
    public VucamControl Vucam = new VucamControl();
    public DriveControl Drive = new DriveControl();
    public ClaimerControl Claimer = new ClaimerControl();
    public MinerControl Miner = new MinerControl();
    public LiftControl Lift = new LiftControl();
    public LedControl Led = new LedControl();
    public ColorControl Color = new ColorControl();

    public boolean doubleSample = false;


    /* Subassembly constructor */
    public WelControl() {
    }

    public void init(LinearOpMode opMode) {
        HardwareMap hwMap;

        opMode.telemetry.addLine("Drive Control" + " initialize");
        opMode.telemetry.update();

        /* Set local copies from opmode class */
        opmode = opMode;
        hwMap = opMode.hardwareMap;

        Vucam.init(opMode);
        Drive.init(opMode);
        Claimer.init(opMode);
        Miner.init(opMode);
        Lift.init(opMode);
        Led.init(opMode);
    }


    public void moveForward(double speed, double time) {
        Drive.moveForward(speed);
        TimeDelay(time);
        Drive.stop();
    }

    public void moveBackward(double speed, double time) {
        Drive.moveBackward(speed);
        TimeDelay(time);
        Drive.stop();
    }

    public void tankDrive(double leftSpeed, double rightSpeed, double time) {
        Drive.tankLeftForward(leftSpeed);
        Drive.tankRightForward(rightSpeed);
        TimeDelay(time);
        Drive.stop();
    }

    public void turnLeft(double speed, double time) {
        Drive.turnLeft(speed);
        TimeDelay(time);
        Drive.stop();
    }

    public void turnRight(double speed, double time) {
        Drive.turnRight(speed);
        TimeDelay(time);
        Drive.stop();
    }

    public void turn2Angle(double speed, double angle) {

        /* P-Control */
        double Max_Speed = 0.55;
        double Min_Speed = 0.10;
        double Inflection_Pt = 37;
        double Pwr_Drop_Strength = 0.2;

        /* D-Control */
        double Damp_Strength = 0.01;

        double angle2turn;

        double start = 0;
        double now = 0;
        double InTargetRange = 0;
        double TimeInTargetRange = 0;
        double interval = 0;
        start = runtime.seconds();

        do {
            Drive.imu.update();
            angle2turn = (angle - Drive.imu.trueAngle);

            Lift.AutoStop();
            Miner.AutoMinerStop();

            if (angle2turn > 180) {
                angle2turn -= 360;
            }
            if (angle2turn < -180) {
                angle2turn += 360;
            }
            //
            now = runtime.seconds() - start;
            if (now >= interval) {
                interval += 0.1;
                opmode.telemetry.addData("trueAngle", Drive.imu.trueAngle);
                opmode.telemetry.update();
            }
            //
            if (angle2turn < 1.0 && angle2turn > -1.0) {
                if (InTargetRange == 0) {
                    InTargetRange = now;
                } else {
                    TimeInTargetRange = now - InTargetRange;
                }
            } else {
                InTargetRange = 0;
                TimeInTargetRange = 0;
            }

            angle2turn = (angle - Drive.imu.trueAngle);
            if (angle2turn > 0.125) {

                Drive.turnRight( ((Max_Speed - Min_Speed)                                                      /
                        (1 + 1.5 * Math.pow(2.71828, (-Pwr_Drop_Strength/5) * (angle2turn - Inflection_Pt)))   +
                        Min_Speed)   * speed   -   (Math.pow(now, 1) * Damp_Strength / 100)
                );

            }
            else if (angle2turn < -0.125) {

                Drive.turnLeft(  ((Max_Speed - Min_Speed)                                                      /
                        (1 + 1.5 * Math.pow(2.71828, (-Pwr_Drop_Strength/5) * (-angle2turn - Inflection_Pt)))   +
                        Min_Speed)   * speed   -   (Math.pow(now, 1) * Damp_Strength / 100)
                );

            }
            else {
                Drive.stop();
                //turnLeft(angle2turn);
            }
        }
        while (TimeInTargetRange < 0.1 && (now > 5.0 || (angle2turn > 5 || angle2turn < -5)) && !opmode.isStopRequested() && opmode.opModeIsActive());
        Drive.stop();
    }

    public void turn2Angle4time(double speed, double angle, double time) {

        /* P-Control */
        double Max_Speed = 0.55;
        double Min_Speed = 0.10;
        double Inflection_Pt = 37;
        double Pwr_Drop_Strength = 0.2;

        /* D-Control */
        double Damp_Strength = 0.01;

        double angle2turn;

        double start = 0;
        double now = 0;
        double InTargetRange = 0;
        double TimeInTargetRange = 0;
        double interval = 0;
        start = runtime.seconds();

        do {
            Drive.imu.update();
            angle2turn = (angle - Drive.imu.trueAngle);

            Lift.AutoStop();
            Miner.AutoMinerStop();

            if (angle2turn > 180) {
                angle2turn -= 360;
            }
            if (angle2turn < -180) {
                angle2turn += 360;
            }
            //
            now = runtime.seconds() - start;
            if (now >= interval) {
                interval += 0.1;
                opmode.telemetry.addData("trueAngle", Drive.imu.trueAngle);
                opmode.telemetry.update();
            }
            //
            if (angle2turn < 1.0 && angle2turn > -1.0) {
                if (InTargetRange == 0) {
                    InTargetRange = now;
                } else {
                    TimeInTargetRange = now - InTargetRange;
                }
            } else {
                InTargetRange = 0;
                TimeInTargetRange = 0;
            }

            angle2turn = (angle - Drive.imu.trueAngle);
            if (angle2turn > 0.125) {

                Drive.turnRight( ((Max_Speed - Min_Speed)                                                      /
                        (1 + 1.5 * Math.pow(2.71828, (-Pwr_Drop_Strength/5) * (angle2turn - Inflection_Pt)))   +
                        Min_Speed)   * speed   -   (Math.pow(now, 1) * Damp_Strength / 100)
                );

            }
            else if (angle2turn < -0.125) {

                Drive.turnLeft(  ((Max_Speed - Min_Speed)                                                      /
                        (1 + 1.5 * Math.pow(2.71828, (-Pwr_Drop_Strength/5) * (-angle2turn - Inflection_Pt)))   +
                        Min_Speed)   * speed   -   (Math.pow(now, 1) * Damp_Strength / 100)
                );

            }
            else {
                Drive.stop();
                //turnLeft(angle2turn);
            }
        }
        while (TimeInTargetRange < 0.1 && now < time && !opmode.isStopRequested() && opmode.opModeIsActive());
        Drive.stop();
    }

    public void turnAngle(double speed, double angle) {
        Drive.imu.update();
        turn2Angle(speed, angle + Drive.imu.trueAngle);
    }

    public void forwardUntilDistance(double speed, double distance) {

        /* P-Control */
        double Max_Speed = 0.75;
        double Min_Speed = 0.07;
        double Inflection_Pt = 50;
        double Pwr_Drop_Strength = 0.2;

        /* D-Control */
        double Damp_Strength = 0.25;

        double distance2drive;
        double start = 0;
        double now = 0;
        double InTargetRange = 0;
        double TimeInTargetRange = 0;
        double interval = 0;
        start = runtime.seconds();

        do {
            //
            Drive.Tof.update();
            distance2drive = (Drive.Tof.TrueDistance - distance);

            Lift.AutoStop();
            Miner.AutoMinerStop();

            now = runtime.seconds() - start;
            if (now >= interval) {
                interval += 0.1;
                opmode.telemetry.addLine("Distance3: " + Drive.Tof.getDistance3());
                opmode.telemetry.update();
            }
            //

            if (distance2drive < 0.8 && distance2drive > -0.8) {
                if (InTargetRange == 0) {
                    InTargetRange = now;
                } else {
                    TimeInTargetRange = now - InTargetRange;
                }
            } else {
                InTargetRange = 0;
                TimeInTargetRange = 0;
            }

            if (distance2drive > 0.125) {

                Drive.moveForward( ((Max_Speed - Min_Speed)                                                      /
                        (1 + 1.5 * Math.pow(2.71828, (-Pwr_Drop_Strength/5) * (distance2drive - Inflection_Pt)))   +
                        Min_Speed)   * speed   -   (Math.pow(now, 1.5) * Damp_Strength / 100)
                );

            }
            else if (distance2drive < -0.125) {

                Drive.moveBackward(  ((Max_Speed - Min_Speed)                                                      /
                        (1 + 1.5 * Math.pow(2.71828, (-Pwr_Drop_Strength/5) * (-distance2drive - Inflection_Pt)))   +
                        Min_Speed)   * speed   -   (Math.pow(now, 1.5) * Damp_Strength / 100)
                );

            }
            else {
                Drive.stop();
                //moveBackward(distance2drive);
            }
        }
        while (TimeInTargetRange < 0.1 && (now > 5.0 || (distance2drive > 5 || distance2drive < 5)) && !opmode.isStopRequested() && opmode.opModeIsActive());
        Drive.stop();
    }

    public void forwardUntilDistance4Time(double speed, double distance, double time) {

        /* P-Control */
        double Max_Speed = 0.75;
        double Min_Speed = 0.07;
        double Inflection_Pt = 50;
        double Pwr_Drop_Strength = 0.2;

        /* D-Control */
        double Damp_Strength = 0.25;

        double distance2drive;
        double start = 0;
        double now = 0;
        double InTargetRange = 0;
        double TimeInTargetRange = 0;
        double interval = 0;
        start = runtime.seconds();

        do {
            //
            Drive.Tof.update();
            distance2drive = (Drive.Tof.TrueDistance - distance);

            Lift.AutoStop();
            Miner.AutoMinerStop();

            now = runtime.seconds() - start;

            if (now >= interval) {
                interval += 0.1;
                opmode.telemetry.addLine("Distance3: " + Drive.Tof.getDistance3());
                opmode.telemetry.update();
            }
            //

            if (distance2drive < 0.8 && distance2drive > -0.8) {
                if (InTargetRange == 0) {
                    InTargetRange = now;
                } else {
                    TimeInTargetRange = now - InTargetRange;
                }
            } else { InTargetRange = 0;
                TimeInTargetRange = 0;
            }

            if (distance2drive > 0.125) {

                Drive.moveForward( ((Max_Speed - Min_Speed)                                                      /
                        (1 + 1.5 * Math.pow(2.71828, (-Pwr_Drop_Strength/5) * (distance2drive - Inflection_Pt)))   +
                        Min_Speed)   * speed   -   ((now - 1) * (now -1) * Damp_Strength / 100)
                );

            }
            else if (distance2drive < -0.125) {

                Drive.moveBackward(  ((Max_Speed - Min_Speed)                                                      /
                        (1 + 1.5 * Math.pow(2.71828, (-Pwr_Drop_Strength/5) * (-distance2drive - Inflection_Pt)))   +
                        Min_Speed)   * speed   -   ((now - 1) * (now -1) * Damp_Strength / 100)
                );

            }
            else {
                Drive.stop();
                //moveBackward(distance2drive);
            }
        }
        while (TimeInTargetRange < 0.1 && now < time && !opmode.isStopRequested() && opmode.opModeIsActive());
        Drive.stop();
    }

    public void DriveUntilColor(double speedfast, double speedslow) {

        double distance;

        while (((Color.getBlue() < Color.COLOR_THRESHOLD) && (Color.getRed() < Color.COLOR_THRESHOLD))) {
            opmode.telemetry.addLine("redV: " + Color.getRed());
            opmode.telemetry.addLine("blueV: " + Color.getBlue());
            opmode.telemetry.update();
            distance = Drive.Tof.getDistance3();
            if (distance > 90) {
                Drive.moveForward(speedfast);
            } else {
                Drive.moveForward(speedslow);
            }

            Lift.AutoStop();
            Miner.AutoMinerStop();
        }
        moveBackward(0.25, 0.05);
        Drive.stop();
    }

    public void DriveUntilTilt(double speed) {
        Drive.imu.setStartAngle2();
        Drive.imu.update();
        while (Drive.imu.trueAngle2 < 10) {
            Lift.AutoStop();
            Miner.AutoMinerStop();

            Drive.imu.update();
            if  (Drive.imu.trueAngle2<10) {
                Drive.moveBackward(speed);
            }
            else
                Drive.stop();
        }
    }

    public void TimeDelay(double time) {
        double start = 0;
        double now = 0;
        start = runtime.seconds();
        do {
            now = runtime.seconds() - start;

            Drive.imu.update();

            Lift.AutoStop();
            Miner.AutoMinerStop();

        } while ((now < time) && !opmode.isStopRequested());

        opmode.telemetry.addLine("trueAngle: " + Drive.imu.trueAngle);
        opmode.telemetry.addLine("Distance: " + Drive.Tof.getDistance3());
        opmode.telemetry.update();


    }
}
