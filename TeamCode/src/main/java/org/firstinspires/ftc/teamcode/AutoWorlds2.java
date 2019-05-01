package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SubAssembly.DriveTrain.DriveUntilColorTest;
import org.firstinspires.ftc.teamcode.SubAssembly.Miner.MinerControl;
import org.firstinspires.ftc.teamcode.SubAssembly.Vucam.VucamControl;
import org.firstinspires.ftc.teamcode.SubAssembly.WelControl;
import org.firstinspires.ftc.teamcode.Utilities.UserControl;
import org.firstinspires.ftc.teamcode.SubAssembly.DriveTrain.DriveControl;
import org.firstinspires.ftc.teamcode.SubAssembly.Lift.LiftControl;
import org.firstinspires.ftc.teamcode.SubAssembly.Claimer.ClaimerControl;
import org.firstinspires.ftc.teamcode.Utilities.AutoTransitioner;
import org.firstinspires.ftc.teamcode.SubAssembly.Leds.LedControl;
import org.firstinspires.ftc.teamcode.SubAssembly.Sensors.ColorControl;

@Autonomous(name = "AutoWorlds2", group = "Auto")
public class AutoWorlds2 extends LinearOpMode {

    /* Sub assemblies */
    WelControl Wel = new WelControl();
    UserControl User = new UserControl();

    //time based variables
    private ElapsedTime runtime = new ElapsedTime();
    private double lastReset = 0;
    private double now = 0;

    /* Constants (set at start of match) */
    double iSpeed = 1.0;

    //State changing a
    // rray
    private void newState(State newState) {
        mCurrentState = newState;
        Wel.Drive.stop();
        Wel.Drive.TimeDelay(0.1);
        resetClock();
    }

    public void resetClock() {
        //resets the clock
        lastReset = runtime.seconds();
    }

    /* Variables go here */

    //State selection variable options
    private enum State {
        Initial,
        Deploy,
        Sample,
        Depot_to_Claim,
        Crater_to_Claim,
        Double_Sample,
        Claim,
        Depot_to_Crater,
        Start_Depot_to_Crater,
        Stop
    }

    //D = Double / R = Right / L = Left / C = Center /
    //Start position variable options
    private enum Start {
        Crater,
        Depot
    }

    //Enum variables (creates variables of the enum variable types previously created)
    private State mCurrentState = State.Initial;
    private Start orientation;
    private boolean doubleSample;
    private boolean allianceCrater;
    private boolean BlueTrue;
    private boolean noLift;
    private int timeDelay = 0;


    public void getUserInput() {

        /* Get user information */

        /* Automatically switch to teleop? */
        if (User.getYesNo("Transition to TeleOp?"))
            AutoTransitioner.transitionOnStop(this, "teleop");

        /*Asks whether you are facing the crater or the depot*/
        if (User.getYesNo("Facing the crater?")) {
            orientation = Start.Crater;
            telemetry.addLine("Facing crater");
        } else {
            orientation = Start.Depot;
            telemetry.addLine("Facing depot");
        }
        telemetry.update();

        /*Asks whether you want to double sample or not*/
        /*if (User.getYesNo("Double Sample?")) {
            doubleSample = true;
            Drive.doubleSample = true;
            telemetry.addLine("Double sample");
        } else {
            doubleSample = false;
            Drive.doubleSample = false;
            telemetry.addLine("Single sample");
        }*/
        telemetry.update();

        /*if (User.getLeftRight("Left or Right Crater?")) {
            allianceCrater = true;
            telemetry.addLine("Alliance Crater");
        } else {
            allianceCrater = false;
            telemetry.addLine("Opponent Crater");
        }*/

        telemetry.update();

        /*Asks whether you want to do a time delay and if you do then it lets you set the length of delay in seconds*/

        timeDelay = User.getInt("Time Delay");
        if (timeDelay < 1) {
            telemetry.addLine("No time delay");
        } else {
            telemetry.addData("Time Delay set to ", timeDelay);
        }

        telemetry.update();

        /*Asks whether you are on the red alliance or the blue alliance*/
        /*if (User.getLeftRight("Red or Blue?")) {
            telemetry.addLine("Blue");
            BlueTrue = true;
        } else {
            telemetry.addLine("Red");
            BlueTrue = false;
        }

        telemetry.update();*/

        /*Asks whether you want to use the lift or not*/
        if (User.getYesNo("Remove Lift?")) {
            telemetry.addLine("No Lift");
            noLift = true;
        } else {
            telemetry.addLine("Yes Lift");
            noLift = false;
        }
        telemetry.update();

    }

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.setAutoClear(false);
        telemetry.addLine("Autonomous");
        telemetry.update();

        /* initialize sub-assemblies
         */
        Wel.init(this);
        User.init(this);

        Wel.Miner.IntakeLower();

        getUserInput();

        //waits for that giant PLAY button to be pressed on RC
        telemetry.addLine(">> Press PLAY to start");
        telemetry.update();
        telemetry.setAutoClear(true);
        waitForStart();

        telemetry.update();
        telemetry.setAutoClear(false);

        VoltageSensor vs = hardwareMap.voltageSensor.get("Expansion Hub 2");
        double Voltage = vs.getVoltage();
        telemetry.addData("Voltage", Voltage);
        iSpeed = (12.7 - Voltage + 9) / 9;

        newState(State.Initial);
        Wel.Drive.imu.setStartAngle();

        while (opModeIsActive() && mCurrentState != State.Stop) {

            now = runtime.seconds() - lastReset;

            Wel.Drive.imu.update();

            Wel.Drive.Tof.Telemetry();

            /*if (mCurrentState != State.Deploy && Lift.LifterButtonB.isPressed()) {
                Lift.Stop();
            }*/

//            telemetry.addData("startAngle", Drive.imu.startAngle);
//            telemetry.addData("currentAngle", Drive.imu.currentAngle);
//            telemetry.addData("trueAngle", Drive.imu.trueAngle);
//            telemetry.update();

            //state switch
            //needs rearranged so that initial is starting state, not land
            switch (mCurrentState) {
                /*Initializes auto and waits for the time delay*/
                case Initial:
                    Wel.Led.white();
                    telemetry.addLine("Initial");
                    telemetry.update();
                    if (now > timeDelay)
                        newState(State.Deploy);
                    break;

                case Deploy:
                    Wel.Led.red();

                    /*Extends the lift until the limit switch is pressed*/
                    if (noLift == false) {
                        telemetry.addLine("Land");
                        telemetry.update();
                        Wel.Miner.Extend();
                        Wel.Drive.TimeDelay(0.5);
                        Wel.Miner.MinerStop();
                        while (!Wel.Lift.LifterButtonT.isPressed()) {
                            Wel.Lift.Extend();
                        }
                        Wel.Lift.Retract();
                        Wel.Miner.Retract();
                    } else {
                        telemetry.addLine("Lift Removed");
                    }

                    /*Tells you whether it is left, right, or center*/
                    Wel.Vucam.setSamplePos();
                    if (Wel.Vucam.sample == VucamControl.Sample.LEFT) {
                        telemetry.addLine("Left");
                    } else if (Wel.Vucam.sample == VucamControl.Sample.RIGHT) {
                        telemetry.addLine("Right");
                    } else if (Wel.Vucam.sample == VucamControl.Sample.CENTER) {
                        telemetry.addLine("Center");
                    } else {
                        telemetry.addLine("Unknown");
                    }
                    telemetry.update();

                    Wel.moveForward(0.5, 0.14);
                    sleep(100);
                    newState(State.Sample);
                    break;

                case Sample:
                    Wel.Led.yellow();
                    telemetry.addLine("Move to crater");
                    telemetry.update();
                    /*Turns and gets correct sample and backs back up*/
                    if (Wel.Vucam.sample == Wel.Vucam.sample.LEFT) {
                        Wel.turn2Angle(iSpeed, -45);
                        Wel.Miner.Extend();
                        Wel.Miner.Intake();
                        Wel.TimeDelay(1.5);
                        Wel.Miner.Retract();
                        Wel.Miner.Stoptake();
                        Wel.TimeDelay(0.5);
                        /*Wel.TimeDelay(0.1);
                        Wel.moveForward(0.5, 0.7);
                        Wel.TimeDelay(0.1);
                        Wel.moveBackward(0.5, 0.5);*/
                    } else if (Wel.Vucam.sample == Wel.Vucam.sample.CENTER) {
                        /*Wel.moveForward(0.5, 0.4);
                        Wel.TimeDelay(0.1);
                        Wel.moveBackward(0.5, 0.3);*/
                        Wel.Miner.Extend();
                        Wel.Miner.Intake();
                        Wel.TimeDelay(1.2);
                        Wel.Miner.Retract();
                        Wel.Miner.Stoptake();
                        Wel.TimeDelay(0.3);
                    } else {
                        Wel.turn2Angle(iSpeed, 45);
                        Wel.Miner.Extend();
                        Wel.Miner.Intake();
                        Wel.TimeDelay(1.5);
                        Wel.Miner.Retract();
                        Wel.Miner.Stoptake();
                        Wel.TimeDelay(0.5);
                        /*Wel.TimeDelay(0.1);
                        Wel.moveForward(0.5, 0.7);
                        Wel.TimeDelay(0.1);
                        Wel.moveBackward(0.5, 0.5);*/
                    }

                    // Navigate to wall on the side of the crater
                    Wel.turn2Angle(iSpeed, -85);
                    Wel.TimeDelay(0.1);
                    Wel.moveForward(0.3, 1.2);
                    Wel.turn2Angle4time(iSpeed,-42, 2.0);
                    Wel.forwardUntilDistance4Time(iSpeed, 20, 4);

                    if (orientation == Start.Depot) {
                        newState(State.Depot_to_Claim);
                    } else if (orientation == Start.Crater) {
                        newState(State.Crater_to_Claim);
                    }
                    break;

                case Depot_to_Claim:
                    /*Manuvers from by the crater to the depot*/
                    telemetry.addLine("Move to depot");
                    telemetry.update();

                    Wel.TimeDelay(0.1);
                    Wel.turn2Angle(iSpeed, 45);
                    Wel.TimeDelay(0.1);
                    Wel.moveForward(0.75, 0.4);
                    Wel.DriveUntilColor(0.25, 0.20);
                    newState(State.Claim);
                    break;

                case Crater_to_Claim:
                    /*Manuvers from by the crater to the depot*/
                    telemetry.addLine("Move to depot");
                    telemetry.update();

                    Wel.TimeDelay(0.1);
                    Wel.turn2Angle(iSpeed, -135);
                    Wel.TimeDelay(0.1);
                    Wel.moveForward(0.75, 0.6);
                    Wel.DriveUntilColor(0.25, 0.20);
                    if (doubleSample) {
                        newState(State.Double_Sample);
                    } else {
                        newState(State.Claim);
                    }
                    break;

                case Double_Sample:
                    /*Navigation to the second sample to double sample*/
                    telemetry.addLine("Double sample");
                    telemetry.update();
                    if (Wel.Vucam.sample == Wel.Vucam.sample.LEFT) {
                        Wel.turn2Angle4time(iSpeed, -60, 4);
                        Wel.Claimer.reset();
                        Wel.TimeDelay(0.1);
                        Wel.moveBackward(0.8, 0.45);
                        Wel.TimeDelay(0.1);
                        Wel.forwardUntilDistance4Time(iSpeed, 13.5, 5);
                    } else if (Wel.Vucam.sample == Wel.Vucam.sample.CENTER) {
                        Wel.turn2Angle4time(iSpeed, -90, 4);
                        Wel.Claimer.reset();
                        Wel.TimeDelay(0.1);
                        Wel.moveBackward(0.8, 0.4);
                        Wel.TimeDelay(0.1);
                        Wel.forwardUntilDistance4Time(iSpeed, 21, 6);
                    } else {
                        Wel.turn2Angle4time(iSpeed, -120, 4);
                        Wel.Claimer.reset();
                        Wel.TimeDelay(0.1);
                        Wel.moveBackward(0.8, 0.45);
                        Wel.TimeDelay(0.1);
                        Wel.forwardUntilDistance4Time(iSpeed, 13.5, 5);
                    }
                    newState(State.Claim);
                    break;

                case Claim:
                    Wel.Led.lawnGreen();
                    telemetry.addLine("Claim");
                    telemetry.update();
                    if (orientation == Start.Depot) {
                        Wel.turn2Angle(iSpeed, -25.0);
                        Wel.Claimer.drop();
                        Wel.TimeDelay(0.5);
                        Wel.Claimer.reset();
                        newState(State.Start_Depot_to_Crater);
                    } else {
                        Wel.turn2Angle(iSpeed, -135.0);
                        Wel.Claimer.drop();
                        Wel.TimeDelay(0.5);
                        Wel.Claimer.reset();
                        newState(State.Depot_to_Crater);
                    }
                    break;

                case Depot_to_Crater:
                    Wel.Led.darkBlue();
                    telemetry.addLine("Depot to crater");
                    telemetry.update();
                    /*if (!allianceCrater) {
                        Drive.turn2Angle(TURN_SPEED, -45);
                    }*/
                    Wel.DriveUntilTilt(0.75);
                    //Drive.moveBackward(0.65, 2.0); //dropped for w/ 11454
                    newState(State.Stop);
                    break;

                case Start_Depot_to_Crater:
                    Wel.Led.lawnGreen();
                    telemetry.addLine("Start at Depot, to crater");
                    telemetry.update();
                    Wel.turn2Angle(iSpeed, 45);
                    Wel.DriveUntilTilt(1);
                    //Drive.moveBackward(0.5, 2.7);
                    /*if (!allianceCrater) {
                        Drive.turn2Angle(TURN_SPEED, 45);
                    }

                    if (Vucam.sample == Vucam.sample.LEFT && orientation == Start.Depot && doubleSample && allianceCrater) {
                        Drive.moveBackward(0.5, 1.2);
                        Drive.TimeDelay(0.1);
                        Drive.turn2Angle(TURN_SPEED, -20);
                        Drive.TimeDelay(0.1);
                        Drive.moveBackward(0.4, 0.6);
                        Drive.TimeDelay(0.1);
                        Drive.turn2Angle(TURN_SPEED, -50);
                        Drive.TimeDelay(0.1);
                        Drive.moveBackward(0.5, 0.9);
                    } else {
                        Drive.moveBackward(0.5, 2.7);
                    }*/
                    newState(State.Stop);
                    break;

                case Stop:
                    telemetry.addLine("Stop");
                    telemetry.update();
                    break;

                default:
                    break;
            }
            sleep(40);
        }

        Wel.Drive.stop();

        telemetry.setAutoClear(true);
    }
}
