package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SubAssembly.DriveTrain.DriveUntilColorTest;
import org.firstinspires.ftc.teamcode.SubAssembly.Miner.MinerControl;
import org.firstinspires.ftc.teamcode.SubAssembly.Vucam.VucamControl;
import org.firstinspires.ftc.teamcode.Utilities.UserControl;
import org.firstinspires.ftc.teamcode.SubAssembly.DriveTrain.DriveControl;
import org.firstinspires.ftc.teamcode.SubAssembly.Lift.LiftControl;
import org.firstinspires.ftc.teamcode.SubAssembly.Claimer.ClaimerControl;
import org.firstinspires.ftc.teamcode.Utilities.AutoTransitioner;
import org.firstinspires.ftc.teamcode.SubAssembly.Leds.LedControl;
import org.firstinspires.ftc.teamcode.SubAssembly.Sensors.ColorControl;

@Autonomous(name = "AutoWorlds1", group = "Auto")
public class AutoWorlds extends LinearOpMode {

    /* Sub assemblies */
    UserControl User = new UserControl();
    VucamControl Vucam = new VucamControl();
    DriveControl Drive = new DriveControl();
    ClaimerControl Claimer = new ClaimerControl();
    MinerControl Miner = new MinerControl();
    LiftControl Lift = new LiftControl();
    LedControl Led = new LedControl();
    ColorControl Color = new ColorControl();

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
        Drive.stop();
        Drive.TimeDelay(0.1);
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
        DepotClaim,
        CraterClaim,
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
    private boolean lift;
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
        if (User.getYesNo("Lift?")) {
            telemetry.addLine("Lifting");
            lift = true;
        } else {
            telemetry.addLine("No Lift");
            lift = false;
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
        User.init(this);
        Vucam.init(this);
        Drive.init(this);
        Claimer.init(this);
        Miner.init(this);
        Lift.init(this);
        Led.init(this);

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
        Drive.imu.setStartAngle();

        while (opModeIsActive() && mCurrentState != State.Stop) {

            now = runtime.seconds() - lastReset;

            Drive.imu.update();

            Drive.Tof.Telemetry();

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
                    Led.white();
                    telemetry.addLine("Initial");
                    telemetry.update();
                    if (now > timeDelay)
                        //newState(State.Deploy);
                        newState(State.Start_Depot_to_Crater);
                    break;

                case Deploy:
                    Led.red();

                    /*Extends the lift until the limit switch is pressed*/
                    if (lift == true) {
                        Miner.Extend4Time(0.28);
                        telemetry.addLine("Land");
                        telemetry.update();
                        Lift.Extend();
                        Drive.TimeDelay(0.25);
                        while (!Lift.LifterButtonT.isPressed()) {
                            Lift.Extend();
                        }
                        Lift.Stop();
                        while (!Miner.MinerButtonI.isPressed()) {
                            Miner.Retract4Time(0.5);
                        }
                    } else {
                        telemetry.addLine("Lift Removed");
                    }

                    /*Tells you whether it is left, right, or center*/
                    Vucam.setSamplePos();
                    if (Vucam.sample == VucamControl.Sample.LEFT) {
                        telemetry.addLine("Left");
                    } else if (Vucam.sample == VucamControl.Sample.RIGHT) {
                        telemetry.addLine("Right");
                    } else if (Vucam.sample == VucamControl.Sample.CENTER) {
                        telemetry.addLine("Center");
                    } else {
                        telemetry.addLine("Unknown");
                    }
                    telemetry.update();

                    //Miner.Retract();
                    Drive.moveForward(0.5 * iSpeed, 0.14);
                    sleep(100);
                    newState(State.Sample);
                    break;

                case Sample:
                    Led.yellow();
                    telemetry.addLine("Move to crater");
                    telemetry.update();
                    /*Turns and gets correct sample and backs back up*/
                    if (Vucam.sample == Vucam.sample.LEFT) {
                        Drive.turn2Angle(iSpeed, -45);
                        Drive.TimeDelay(0.1);
                        Drive.moveForward(0.5 * iSpeed, 0.7);
                        Drive.TimeDelay(0.1);
                        Drive.moveBackward(0.5 * iSpeed, 0.5);
                    } else if (Vucam.sample == Vucam.sample.CENTER) {
                        Drive.moveForward(0.5, 0.4);
                        Drive.TimeDelay(0.1);
                        Drive.moveBackward(0.5 * iSpeed, 0.3);
                    } else {
                        Drive.turn2Angle(iSpeed, 45);
                        Drive.TimeDelay(0.1);
                        Drive.moveForward(0.5 * iSpeed, 0.7);
                        Drive.TimeDelay(0.1);
                        Drive.moveBackward(0.5 * iSpeed, 0.5);
                    }

                    if (orientation == Start.Depot) {

                        Drive.turn2Angle4time(iSpeed, -93,3);
                        Drive.TimeDelay(0.1);
                        Drive.moveForward(0.3 * iSpeed, 1.4);
                        Drive.turn2Angle4time(iSpeed, -45, 2.0);
                        Drive.forwardUntilDistance4Time(iSpeed - 0.07, 5, 4);

                        newState(State.DepotClaim);

                    } else if (orientation == Start.Crater) {

                        // Navigate to wall on the side of the crater
                        Drive.turn2Angle(iSpeed, -88);
                        Drive.TimeDelay(0.1);
                        Drive.moveForward(0.3 * iSpeed, 1.5);
                        Drive.turn2Angle4time(iSpeed, -45, 2.0);
                        Drive.forwardUntilDistance4Time(iSpeed, 6, 4);

                        newState(State.CraterClaim);
                    }
                    break;

                case DepotClaim:
                    /*Manuvers to the depot*/
                    telemetry.addLine("Move to depot");
                    telemetry.update();

                    Drive.TimeDelay(0.1);
                    Drive.turn2Angle(iSpeed, 43);
                    Drive.TimeDelay(0.1);
                    Drive.moveForward(0.75 * iSpeed, 0.2);
                    Drive.DriveUntilColor(0.25 * iSpeed, 0.20 * iSpeed);
                    newState(State.Claim);

                    break;

                case CraterClaim:
                    /*Manuvers from by the crater to the depot*/
                    telemetry.addLine("Move to depot");
                    telemetry.update();

                    Drive.TimeDelay(0.1);
                    Drive.turn2Angle(iSpeed, -130);
                    Drive.TimeDelay(0.1);
                    Drive.moveForward(0.75 * iSpeed, 0.6);
                    Drive.DriveUntilColor(0.25 * iSpeed, 0.20 * iSpeed);
                    if (doubleSample) {
                        Claimer.drop();
                        Drive.TimeDelay(0.5);
                        Claimer.reset();
                        newState(State.Double_Sample);
                    } else {
                        newState(State.Claim);
                    }
                    break;

                case Double_Sample:
                    /*Navigation to the second sample to double sample*/
                    telemetry.addLine("Double sample");
                    telemetry.update();
                    if (Vucam.sample == Vucam.sample.LEFT) {
                        Drive.turn2Angle4time(iSpeed, -10, 4);
                        Drive.TimeDelay(0.1);
                        Drive.moveBackward(0.8 * iSpeed, 0.4);
                    } else if (Vucam.sample == Vucam.sample.CENTER) {
                        Drive.turn2Angle4time(iSpeed, -60, 4);
                        Drive.TimeDelay(0.1);
                        Drive.moveBackward(0.8 * iSpeed, 0.3);
                    } else {
                        Drive.turn2Angle4time(iSpeed, -100, 4);
                        Drive.TimeDelay(0.1);
                        Drive.moveBackward(0.8 * iSpeed, 0.25);
                    }
                    Drive.TimeDelay(0.1);
                    Drive.turn2Angle4time(iSpeed, -45, 3);
                    Drive.TimeDelay(0.1);
                    Drive.forwardUntilDistance4Time(iSpeed, 6, 4);
                    newState(State.Claim);
                    break;

                case Claim:
                    Led.lawnGreen();
                    telemetry.addLine("Claim");
                    telemetry.update();
                    if (orientation == Start.Depot) {
                        Drive.turn2Angle4time(iSpeed + 0.08, 0, 1.5);
                        Claimer.drop();
                        Drive.TimeDelay(0.5);
                        Claimer.reset();
                        newState(State.Start_Depot_to_Crater);
                    } else {
                        Drive.turn2Angle(iSpeed, -135.0);
                        Claimer.drop();
                        Drive.TimeDelay(0.5);
                        Claimer.reset();
                        newState(State.Depot_to_Crater);
                    }
                    break;

                case Depot_to_Crater:
                    Led.darkBlue();
                    telemetry.addLine("Depot to crater");
                    telemetry.update();
                    Drive.turn2Angle4time(iSpeed - 0.05, -136, 1);
                    Drive.DriveUntilTilt(0.75 * iSpeed);

                    Miner.Extend4Time(0.5);
                    Lift.Retract();
                    Drive.TimeDelay(0.1);
                    do {
                        if (Lift.LifterButtonB.isPressed())  {
                            Lift.Stop();
                        }
                    } while (runtime.seconds() < 29.0);
                    Lift.Stop();
                    newState(State.Stop);
                    break;

                case Start_Depot_to_Crater:
                    Led.lawnGreen();
                    telemetry.addLine("Start at Depot, to crater");
                    telemetry.update();
                    Drive.turn2Angle4time(iSpeed + 0.12, 46, 3);
                    Drive.DriveUntilTilt(0.75 * iSpeed);

                    Miner.Extend4Time(0.5);
                    Lift.Retract();
                    Drive.TimeDelay(0.1);
                    do {
                        if (Lift.LifterButtonB.isPressed())  {
                            Lift.Stop();
                        }
                    } while (runtime.seconds() < 29.0);
                    Lift.Stop();
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

        Drive.stop();

        telemetry.setAutoClear(true);
    }
}