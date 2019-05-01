package org.firstinspires.ftc.teamcode.SubAssembly.DriveTrain;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubAssembly.Sensors.ColorControl;
import org.firstinspires.ftc.teamcode.SubAssembly.Sensors.IMUcontrol;
import org.firstinspires.ftc.teamcode.SubAssembly.Sensors.TofControl;


/* Sub Assembly Class
 */
public class DriveControl {
    /* Declare private class object */
    private LinearOpMode opmode = null;     /* local copy of opmode class */
    private ElapsedTime runtime = new ElapsedTime();

    //initializing motors
    private DcMotor FrontRightM = null;
    private DcMotor FrontLeftM = null;
    private DcMotor BackRightM = null;
    private DcMotor BackLeftM = null;

    public TouchSensor EDist;

    /* Declare public class object */
    public IMUcontrol imu = new IMUcontrol();
    public TofControl Tof = new TofControl();
    public ColorControl Color = new ColorControl();

    public boolean doubleSample = false;


    /* Subassembly constructor */
    public DriveControl() {
    }

    public void init(LinearOpMode opMode) {
        HardwareMap hwMap;

        opMode.telemetry.addLine("Drive Control" + " initialize");
        opMode.telemetry.update();

        /* Set local copies from opmode class */
        opmode = opMode;
        hwMap = opMode.hardwareMap;

        imu.init(opMode);
        Tof.init(opMode);
        Color.init(opMode);

        /* Map hardware devices */
        FrontRightM = hwMap.dcMotor.get("FrontRightM");
        FrontLeftM = hwMap.dcMotor.get("FrontLeftM");
        BackRightM = hwMap.dcMotor.get("BackRightM");
        BackLeftM = hwMap.dcMotor.get("BackLeftM");

        EDist = hwMap.touchSensor.get("EDist");

        //reverses some motors
        BackLeftM.setDirection(DcMotor.Direction.REVERSE);
        FrontLeftM.setDirection(DcMotor.Direction.REVERSE);

        FrontRightM.setPower(0);
        FrontLeftM.setPower(0);
        BackRightM.setPower(0);
        BackLeftM.setPower(0);
    }

    //setting power to move forward
    public void moveForward(double speed) {
        FrontRightM.setPower(speed);
        FrontLeftM.setPower(speed);
        BackRightM.setPower(speed);
        BackLeftM.setPower(speed);
    }

    public void moveForward(double speed, double time) {
        moveForward(speed);
        TimeDelay(time);
        stop();
    }

    //setting power to move backward
    public void moveBackward(double speed) {
        FrontRightM.setPower(-speed);
        FrontLeftM.setPower(-speed);
        BackRightM.setPower(-speed);
        BackLeftM.setPower(-speed);
    }

    public void moveBackward(double speed, double time) {
        moveBackward(speed);
        TimeDelay(time);
        stop();
    }

    //setting power to turn left
    public void turnLeft(double speed) {
        FrontRightM.setPower(speed);
        FrontLeftM.setPower(-speed);
        BackRightM.setPower(speed);
        BackLeftM.setPower(-speed);
    }

    public void turnLeft(double speed, double time) {
        turnLeft(speed);
        TimeDelay(time);
        stop();
    }

    //setting power to turn right
    public void turnRight(double speed) {
        FrontRightM.setPower(-speed);
        FrontLeftM.setPower(speed);
        BackRightM.setPower(-speed);
        BackLeftM.setPower(speed);
    }

    public void turnRight(double speed, double time) {
        turnRight(speed);
        TimeDelay(time);
        stop();
    }

    public void turn2Angle(double speed, double angle) {

        /* P-Control */
        double Max_Speed = 0.55;
        double Min_Speed = 0.11;
        double Inflection_Pt = 37;
        double Pwr_Drop_Strength = 0.2;

        /* D-Control */
        double Damp_Strength = 0.5;

        double angle2turn;

        double start = 0;
        double now = 0;
        double InTargetRange = 0;
        double TimeInTargetRange = 0;
        double interval = 0;
        start = runtime.seconds();

        do {
            imu.update();
            angle2turn = (angle - imu.trueAngle);

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
                opmode.telemetry.addData("trueAngle", imu.trueAngle);
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

            angle2turn = (angle - imu.trueAngle);
            if (angle2turn > 0.125) {

                turnRight( ((Max_Speed - Min_Speed)                                                      /
                        (1 + 1.5 * Math.pow(2.71828, (-Pwr_Drop_Strength/5) * (angle2turn - Inflection_Pt)))   +
                        Min_Speed)   * speed   -   (Math.pow(now, 0.8) * Damp_Strength / ((angle2turn + 5 ) * 1000))
                );

            }
            else if (angle2turn < -0.125) {

                turnLeft(  ((Max_Speed - Min_Speed)                                                      /
                        (1 + 1.5 * Math.pow(2.71828, (-Pwr_Drop_Strength/5) * (-angle2turn - Inflection_Pt)))   +
                        Min_Speed)   * speed   -   (Math.pow(now, 0.8) * Damp_Strength / ((-angle2turn + 5 ) * 1000))
                );

            }
            else {
                stop();
                //turnLeft(angle2turn);
            }
        }
        while (TimeInTargetRange < 0.1 && (now > 5.0 || (angle2turn > 5 || angle2turn < -5)) && !opmode.isStopRequested() && opmode.opModeIsActive());
        stop();
    }

    public void turn2Angle4time(double speed, double angle, double time) {

        /* P-Control */
        double Max_Speed = 0.55;
        double Min_Speed = 0.11;
        double Inflection_Pt = 37;
        double Pwr_Drop_Strength = 0.2;

        /* D-Control */
        double Damp_Strength = 0.5;

        double angle2turn;

        double start = 0;
        double now = 0;
        double InTargetRange = 0;
        double TimeInTargetRange = 0;
        double interval = 0;
        start = runtime.seconds();

        do {
            imu.update();
            angle2turn = (angle - imu.trueAngle);

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
                opmode.telemetry.addData("trueAngle", imu.trueAngle);
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

            angle2turn = (angle - imu.trueAngle);
            if (angle2turn > 0.125) {

                turnRight( ((Max_Speed - Min_Speed)                                                      /
                        (1 + 1.5 * Math.pow(2.71828, (-Pwr_Drop_Strength/5) * (angle2turn - Inflection_Pt)))   +
                        Min_Speed)   * speed   -   (Math.pow(now, 0.8) * Damp_Strength / ((angle2turn + 5 ) * 1000))
                );

            }
            else if (angle2turn < -0.125) {

                turnLeft(  ((Max_Speed - Min_Speed)                                                      /
                        (1 + 1.5 * Math.pow(2.71828, (-Pwr_Drop_Strength/5) * (-angle2turn - Inflection_Pt)))   +
                        Min_Speed)   * speed   -   (Math.pow(now, 0.8) * Damp_Strength / ((-angle2turn + 5 ) * 1000))
                );

            }
            else {
                stop();
                //turnLeft(angle2turn);
            }
        }
        while (TimeInTargetRange < 0.1 && now < time && !opmode.isStopRequested() && opmode.opModeIsActive());
        stop();
    }

    public void turnAngle(double speed, double angle) {
        imu.update();
        turn2Angle(speed, angle + imu.trueAngle);
    }

    //setting power to 0
    public void stop() {
        FrontRightM.setPower(0);
        FrontLeftM.setPower(0);
        BackRightM.setPower(0);
        BackLeftM.setPower(0);
    }

    public void tankDrive(double leftSpeed, double rightSpeed, double time) {
        tankLeftForward(leftSpeed);
        tankRightForward(rightSpeed);
        TimeDelay(time);
        stop();
    }

    public void tankRightForward(double speed) {
        FrontRightM.setPower(speed);
        BackRightM.setPower(speed);
    }

    public void tankRightBackward(double speed) {
        FrontRightM.setPower(-speed);
        BackRightM.setPower(-speed);
    }

    public void tankLeftForward(double speed) {
        FrontLeftM.setPower(speed);
        BackLeftM.setPower(speed);
    }

    public void tankLeftBackward(double speed) {
        FrontLeftM.setPower(-speed);
        BackLeftM.setPower(-speed);
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
            now = runtime.seconds() - start;

            if (now >= interval) {
                interval += 0.1;
                opmode.telemetry.addLine("Distance3: " + Tof.getDistance3());
                opmode.telemetry.update();
            }
            //
            distance2drive = (Tof.getDistance3() - distance);

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

            if (distance2drive > 0.125 && !EDist.isPressed()) {

                moveForward( ((Max_Speed - Min_Speed)                                                      /
                        (1 + 1.5 * Math.pow(2.71828, (-Pwr_Drop_Strength/5) * (distance2drive - Inflection_Pt)))   +
                        Min_Speed)   * speed   -   (Math.pow(now, 1.5) * Damp_Strength / ((distance2drive + 5 ) * 10))
                );

            }
            else if (distance2drive < -0.125) {

                moveBackward(  ((Max_Speed - Min_Speed)                                                      /
                        (1 + 1.5 * Math.pow(2.71828, (-Pwr_Drop_Strength/5) * (-distance2drive - Inflection_Pt)))   +
                        Min_Speed)   * speed   -   (Math.pow(now, 1.5) * Damp_Strength / ((-distance2drive + 5 ) * 10))
                );

            }
            else {
                stop();
                //moveBackward(distance2drive);
            }
        }
        while (TimeInTargetRange < 0.1 && (now > 5.0 || (distance2drive > 5 || distance2drive < 5)) && !opmode.isStopRequested() && opmode.opModeIsActive());
        stop();
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
            now = runtime.seconds() - start;

            if (now >= interval) {
                interval += 0.1;
                opmode.telemetry.addLine("Distance3: " + Tof.getDistance3());
                opmode.telemetry.update();
            }
            //
            distance2drive = (Tof.getDistance3() - distance);

            if (distance2drive < 0.8 && distance2drive > -0.8) {
                if (InTargetRange == 0) {
                    InTargetRange = now;
                } else {
                    TimeInTargetRange = now - InTargetRange;
                }
            } else { InTargetRange = 0;
                TimeInTargetRange = 0;
            }

            if (distance2drive > 0.125 && !EDist.isPressed()) {

                moveForward( ((Max_Speed - Min_Speed)                                                      /
                        (1 + 1.5 * Math.pow(2.71828, (-Pwr_Drop_Strength/5) * (distance2drive - Inflection_Pt)))   +
                        Min_Speed)   * speed   -   ((now - 1) * (now -1) * Damp_Strength / ((distance2drive + 5 ) * 10))
                );

            }
            else if (distance2drive < -0.125) {

                moveBackward(  ((Max_Speed - Min_Speed)                                                      /
                        (1 + 1.5 * Math.pow(2.71828, (-Pwr_Drop_Strength/5) * (-distance2drive - Inflection_Pt)))   +
                        Min_Speed)   * speed   -   ((now - 1) * (now -1) * Damp_Strength / ((-distance2drive + 5 ) * 10))
                );

            }
            else {
                stop();
                //moveBackward(distance2drive);
            }
        }
        while (TimeInTargetRange < 0.1 && now < time && !opmode.isStopRequested() && opmode.opModeIsActive());
        stop();
    }

    public void TimeDelay(double time) {
        double start = 0;
        double now = 0;
        start = runtime.seconds();
        do {
            now = runtime.seconds() - start;

            imu.update();

        } while ((now < time) && !opmode.isStopRequested());

        opmode.telemetry.addLine("trueAngle: " + imu.trueAngle);
        opmode.telemetry.addLine("Distance: " + Tof.getDistance3());
        opmode.telemetry.update();


    }

    public void DriveUntilColor(double speedfast, double speedslow) {

        double distance;

        while (((Color.getBlue() < Color.COLOR_THRESHOLD) && (Color.getRed() < Color.COLOR_THRESHOLD))) {
            opmode.telemetry.addLine("redV: " + Color.getRed());
            opmode.telemetry.addLine("blueV: " + Color.getBlue());
            opmode.telemetry.update();
            distance = Tof.getDistance3();
            if (distance > 90) {
                moveForward(speedfast);
            } else {
                moveForward(speedslow);
            }
        }
        moveBackward(0.25, 0.05);
        stop();
    }

    public void DriveUntilTilt(double speed) {
        imu.setStartAngle2();
        imu.update();
        while (imu.trueAngle2 < 10) {
            imu.update();
            if  (imu.trueAngle2<10) {
                moveBackward(speed);
            }
            else
                stop();
        }
    }

    /*public void DriveUntilColorDistance(double speed) {
        while (((Color.getBlue() < Color.COLOR_THRESHOLD) && (Color.getRed() < Color.COLOR_THRESHOLD)) /*&& ((runtime.seconds() < 26 && !doubleSample) || (runtime.seconds() < 20 && doubleSample))) {
            opmode.telemetry.addLine("redV: " + Color.getRed());
            opmode.telemetry.addLine("blueV: " + Color.getBlue());
            opmode.telemetry.addLine("runtime: " + runtime.seconds());
            opmode.telemetry.update();
            while (Tof.getDistance3() > 10) {
                DriveUntilColor(speed);
            }
        }
        moveBackward(0.25, 0.05);
        stop();
    }*/
}