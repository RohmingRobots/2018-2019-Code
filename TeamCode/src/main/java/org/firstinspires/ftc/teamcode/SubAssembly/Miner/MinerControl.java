package org.firstinspires.ftc.teamcode.SubAssembly.Miner;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Utilities.EnumWrapper;
import org.firstinspires.ftc.teamcode.Utilities.ServoControl;

import java.util.EnumMap;

/* Sub Assembly Class
 */
public class MinerControl {
    /* Constants */
    final double LINEAR_SPEED = 1.0;
    final double INTAKE_SPEED = 0.75;

    /* Declare private class object */
    private LinearOpMode opmode = null;     /* local copy of opmode class */
    public ElapsedTime runtime = new ElapsedTime();


    private DcMotor LinearMinerM;
    private DcMotor IntakeM;
    private Servo MinerLeftS;
    private Servo MinerRightS;
    private Servo DeployerS;

    private EnumMap<Setpoints, Double> MapDepServo;

    public ServoControl<Setpoints, EnumMap<Setpoints, Double>> DeployerServo;

    public enum Setpoints implements EnumWrapper<Setpoints> {
        Undump, Middump, Dump;
    }

    /* Declare public class object */
    public TouchSensor MinerButtonI;
    public TouchSensor MinerButtonO;

    /* Subassembly constructor */
    public MinerControl() {
    }

    public void init(LinearOpMode opMode) {
        init(opMode, true);
    }

    public void init(LinearOpMode opMode, boolean init_servo) {
        HardwareMap hwMap;

        opMode.telemetry.addLine("Lift Control" + " initialize");
        opMode.telemetry.update();

        /* Set local copies from opmode class */
        opmode = opMode;
        hwMap = opMode.hardwareMap;

        MapDepServo = new EnumMap<Setpoints, Double>(Setpoints.class);


        /* Assign setpoint values */
        MapDepServo.put(Setpoints.Undump, 0.15);
        MapDepServo.put(Setpoints.Middump, 0.58);
        MapDepServo.put(Setpoints.Dump, 0.93);

        /* Map hardware devices */
        LinearMinerM = hwMap.dcMotor.get("LinearMinerM");
        IntakeM = hwMap.dcMotor.get("IntakeM");
        MinerLeftS = hwMap.servo.get("MinerLeftS");
        MinerRightS = hwMap.servo.get("MinerRightS");
        MinerButtonI = hwMap.touchSensor.get("MinerButtonI");
        MinerButtonO = hwMap.touchSensor.get("MinerButtonO");
        DeployerS = hwMap.servo.get("DeployerS");

        LinearMinerM.setPower(0);
        IntakeM.setPower(0);
        DeployerS.setPosition(1);

        DeployerServo = new ServoControl(DeployerS, MapDepServo, Setpoints.Undump, true);
    }

    public void Extend() {
        LinearMinerM.setPower(-LINEAR_SPEED);
    }

    public void Extend4Time(double time) {
        Extend();
        TimeDelayExtend(time);
        MinerStop();
    }

    public void Retract() {
        LinearMinerM.setPower(LINEAR_SPEED);
    }

    public void Retract4Time(double time) {
        if (!MinerButtonI.isPressed()) {
            Retract();
            TimeDelayRetract(time);
        } else {
            MinerStop();
        }
        MinerStop();
    }

    public void MinerStop() {
        LinearMinerM.setPower(0);
    }

    public void Untake() {
        IntakeM.setPower(INTAKE_SPEED);
    }

    public void Stoptake() {
        IntakeM.setPower(0);
    }

    public void Intake() {
        IntakeM.setPower(-INTAKE_SPEED);
    }


    public void IntakeRaise() {
        MinerLeftS.setPosition(0.2);
        MinerRightS.setPosition(0.8);
    }

    public void IntakeLower() {
        MinerLeftS.setPosition(0.6);
        MinerRightS.setPosition(0.2);
    }

    public void deployUp() {
        DeployerServo.nextSetpoint(true);
    }

    public void deployDown() {
        DeployerServo.prevSetpoint(true);
    }

    public void autoMine() {
        if (!MinerButtonO.isPressed()) {
            Extend();
        } else if (!MinerButtonI.isPressed()) {
            Retract();
        } else {
            MinerStop();
        }
    }

    public void AutoMinerStop() {
        if (MinerButtonO.isPressed() || MinerButtonI.isPressed()) {
            MinerStop();
        }
    }

    public void TimeDelay(double time) {
        double start = 0;
        double now = 0;
        start = runtime.seconds();
        do {
            now = runtime.seconds() - start;
            AutoMinerStop();
        } while ((now < time) && !opmode.isStopRequested());
    }

    public void TimeDelayExtend(double time) {
        double start = 0;
        double now = 0;
        start = runtime.seconds();
        do {
            now = runtime.seconds() - start;
            if (MinerButtonO.isPressed()) {
                MinerStop();
            }
        } while ((now < time) && !opmode.isStopRequested());
    }

    public void TimeDelayRetract(double time) {
        double start = 0;
        double now = 0;
        start = runtime.seconds();
        do {
            now = runtime.seconds() - start;
            if (MinerButtonI.isPressed()) {
                MinerStop();
            }
        } while ((now < time) && !opmode.isStopRequested());
    }
}