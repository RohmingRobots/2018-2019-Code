package org.firstinspires.ftc.teamcode.SubAssembly.Lift;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubAssembly.Leds.LedControl;

/* Sub Assembly Class
 */
public class LiftControl {
    /* Constants */
    final double LIFT_SPEED = 1.0;

    /* Declare private class object */
    private LinearOpMode opmode = null;     /* local copy of opmode class */

    private DcMotor LifterRightM;
    private DcMotor LifterLeftM;
    private Servo LockRightS;
    private Servo LockLeftS;
    private boolean locked;
    private ElapsedTime runtime = new ElapsedTime();


    /* Declare public class object */
    public TouchSensor LifterButtonT;
    public TouchSensor LifterButtonB;

    /* Subassembly constructor */
    public LiftControl() {
    }

    public void init(LinearOpMode opMode) {
        HardwareMap hwMap;

        opMode.telemetry.addLine("Lift Control" + " initialize");
        opMode.telemetry.update();

        /* Set local copies from opmode class */
        opmode = opMode;
        hwMap = opMode.hardwareMap;


        /* Map hardware devices */
        LifterRightM = hwMap.dcMotor.get("LifterRightM");
        LifterLeftM = hwMap.dcMotor.get("LifterLeftM");
        LockRightS = hwMap.servo.get("LockRightS");
        LockLeftS = hwMap.servo.get("LockLeftS");
        LifterButtonB = hwMap.touchSensor.get("LifterButtonB");
        LifterButtonT = hwMap.touchSensor.get("LifterButtonT");

        Unlock();
        LifterRightM.setPower(0);
        LifterLeftM.setPower(0);
        LifterRightM.setDirection(DcMotor.Direction.REVERSE);
    }

    public void Extend() {
        if (!locked) {
            LifterLeftM.setPower(LIFT_SPEED);
            LifterRightM.setPower(LIFT_SPEED);
        }
    }

    public void Extend4Time(double time) {
        if (!LifterButtonT.isPressed()) {
            Extend();
            TimeDelayExtend(time);
        } else {
            Stop();
        }
        Stop();
    }

    public void Retract() {
        if (!locked) {
            LifterLeftM.setPower(-LIFT_SPEED);
            LifterRightM.setPower(-LIFT_SPEED);
        }
    }

    public void Retract4Time(double time) {
        if (!LifterButtonB.isPressed()) {
            Retract();
            TimeDelayRetract(time);
        } else {
            Stop();
        }
        Stop();
    }

    public void Stop() {
        LifterLeftM.setPower(0);
        LifterRightM.setPower(0);
    }

    public void AutoStop() {
        if (LifterButtonT.isPressed() || LifterButtonB.isPressed()) {
            Stop();
        }
    }

    //locking the servos into place to hold position
    public void Lock() {
        LockRightS.setPosition(0.44);
        LockLeftS.setPosition(0.4);
        locked = true;
    }

    //unlocking the servos
    public void Unlock() {
        LockRightS.setPosition(0.77);
        LockLeftS.setPosition(0.1);
        locked = false;
    }

    public void AutoLift() {
        Unlock();
        if (!LifterButtonT.isPressed()) {
            Extend();
        } else {
            Stop();
        }
    }

    public void TimeDelay(double time) {
        double start = 0;
        double now = 0;
        start = runtime.seconds();
        do {
            now = runtime.seconds() - start;
            AutoStop();
        } while ((now < time) && !opmode.isStopRequested());
    }

    public void TimeDelayExtend(double time) {
        double start = 0;
        double now = 0;
        start = runtime.seconds();
        do {
            now = runtime.seconds() - start;
            if (LifterButtonT.isPressed()) {
                Stop();
            }
        } while ((now < time) && !opmode.isStopRequested());
    }

    public void TimeDelayRetract(double time) {
        double start = 0;
        double now = 0;
        start = runtime.seconds();
        do {
            now = runtime.seconds() - start;
            if (LifterButtonB.isPressed()) {
                Stop();
            }
        } while ((now < time) && !opmode.isStopRequested());
    }
}