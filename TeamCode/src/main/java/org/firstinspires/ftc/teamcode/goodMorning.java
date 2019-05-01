package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubAssembly.DriveTrain.DriveControl;

import org.firstinspires.ftc.teamcode.SubAssembly.Leds.LedControl;
import org.firstinspires.ftc.teamcode.SubAssembly.Lift.LiftControl;
import org.firstinspires.ftc.teamcode.SubAssembly.Miner.MinerControl;
import org.firstinspires.ftc.teamcode.SubAssembly.Sensors.TofControl;
import org.firstinspires.ftc.teamcode.Utilities.GamepadWrapper;

/* Sub Assembly Test OpMode
 * This TeleOp OpMode is used to test the functionality of the specific sub assembly
 */
// Assign OpMode type (TeleOp or Autonomous), name, and grouping
@TeleOp(name = "Good Morning", group = "Drive")
public class goodMorning extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.setAutoClear(false);
        telemetry.addLine("Good Morning");

        double DriveSpeed = 1;
        double TurnSpeed = DriveSpeed / 2;
        int reverse = -1;
        int Manual = 0;
        int LoadState = 0;
        int UnloadState = 0;
        double LoadTimerStart = 0, LoadTimerNow;

        /* initialize sub-assemblies
         */
        LiftControl Lift = new LiftControl();
        TofControl Tof = new TofControl();
        LedControl Led = new LedControl();
        MinerControl Miner = new MinerControl();

        GamepadWrapper egamepad1 = new GamepadWrapper(gamepad1);
        GamepadWrapper egamepad2 = new GamepadWrapper(gamepad2);

        Lift.init(this);
        Tof.init(this);
        Led.init(this);
        Miner.init(this);

        //waits for that giant PLAY button to be pressed on RC
        telemetry.addLine(">> Press PLAY to start");
        telemetry.update();
        telemetry.setAutoClear(true);
        waitForStart();

        //telling the code to run until you press that giant STOP button on RC
        while (opModeIsActive()) {


            egamepad1.updateEdge();
            egamepad2.updateEdge();

            //Ready Player Two

            if ((gamepad2.left_stick_y < -0.4) || (gamepad2.left_stick_y > 0.4) || (egamepad2.dpad_up.pressed) || (egamepad2.dpad_down.pressed) || (egamepad2.left_bumper.pressed) || (egamepad2.left_trigger.pressed)) {
                Manual = 0;
            } else if (egamepad2.a.pressed) {
                Manual = 1;
            } else if (egamepad2.b.pressed) {
                // deployer and lifter down put in manual 2
                if (Miner.DeployerServo.getSetpoint() != MinerControl.Setpoints.Undump) {
                    Manual = 3;
                } else {
                    Manual = 2;
                    LoadState = 0;
                }
            }


            switch (Manual) {
                case 0:         // manual
                    //lift control
                    if ((egamepad2.dpad_up.state) && (!Lift.LifterButtonT.isPressed())) {
                        Led.rainbowRainbowPalette();
                        Lift.Extend();
                    } else if ((egamepad2.dpad_down.state) && (!Lift.LifterButtonB.isPressed())) {
                        Led.rainbowPartyPalette();
                        Lift.Retract();
                    } else {
                        Lift.Stop();
                    }

                    //lift lock controls
                    if (egamepad2.dpad_right.released) {
                        Lift.Lock();
                    } else if (egamepad2.dpad_left.released) {
                        Lift.Unlock();
                    }

                    //miner controls
                    if (egamepad2.y.released) {
                        Miner.IntakeLower();
                        Led.orange();
                    } else if (egamepad2.x.released) {
                        Miner.IntakeRaise();
                        Led.white();
                    }

                    if (egamepad2.left_trigger.released) {
                        Miner.deployDown();
                    } else if (egamepad2.left_bumper.released) {
                        Miner.deployUp();
                    }

                    if (gamepad2.right_stick_y < -0.4) {
                        Miner.Untake();
                    } else if (gamepad2.right_stick_y > 0.4) {
                        Miner.Intake();
                    } else {
                        Miner.Stoptake();
                    }

                    if (gamepad2.left_stick_y < -0.4) {
                        if (!Miner.MinerButtonO.isPressed()) {
                            Miner.Extend();
                        } else {
                            Miner.MinerStop();
                        }
                    } else if (gamepad2.left_stick_y > 0.4) {
                        if (!Miner.MinerButtonI.isPressed()) {
                            Miner.Retract();
                        } else {
                            Miner.MinerStop();
                        }
                    } else {
                        Miner.MinerStop();
                    }
                    break;

                case 1:     // auto in and out
                    if (egamepad2.a.state) {
                        if (!Miner.MinerButtonO.isPressed()) {
                            Miner.Intake();
                            Miner.Extend();
                        } else {
                            Miner.MinerStop();
                        }
                    } else {
                        if (!Miner.MinerButtonI.isPressed()) {
                            Miner.Stoptake();
                            Miner.Retract();
                        } else {
                            Miner.MinerStop();
                            Miner.Stoptake();
                        }
                    }
                    break;

                case 2:     // auto load
                    switch (LoadState) {
                        case 0:
                            if (!Lift.LifterButtonB.isPressed()) {
                                Manual = 0;
                                break;
                            }
                            if (!Miner.MinerButtonI.isPressed()) {
                                Miner.Retract();
                            } else {
                                Miner.MinerStop();
                                LoadState = 1;
                                LoadTimerStart = Miner.runtime.seconds();
                            }
                            break;
                        case 1:
                            Miner.Intake();
                            Miner.IntakeLower();
                            LoadTimerNow = Miner.runtime.seconds() - LoadTimerStart;
                            if (LoadTimerNow > 0.5) {
                                LoadState = 2;
                                LoadTimerStart = Miner.runtime.seconds();
                            }
                            break;
                        case 2:
                            Miner.Stoptake();
                            Miner.Extend();
                            LoadTimerNow = Miner.runtime.seconds() - LoadTimerStart;
                            if (LoadTimerNow > 0.8) {
                                Miner.MinerStop();
                                Miner.DeployerServo.setSetpoint(MinerControl.Setpoints.Middump);
                                LoadState = 3;
                                LoadTimerStart = Miner.runtime.seconds();
                            }
                            break;
                        case 3:
                            Lift.Extend();
                            LoadTimerNow = Miner.runtime.seconds() - LoadTimerStart;
                            if (LoadTimerNow > 1.7) {
                                Lift.Stop();
                                LoadState = -1;
                            }
                            break;
                        default:
                            Manual = 0;
                            break;
                    }
                    break;

                case 3:     // auto unload
                    switch (UnloadState) {
                        case 0:
                            if (Miner.DeployerServo.getSetpoint() == MinerControl.Setpoints.Middump) {
                                Miner.DeployerServo.setSetpoint(MinerControl.Setpoints.Dump);
                                UnloadState = 1;
                                LoadTimerStart = Miner.runtime.seconds();
                            } else if (Miner.DeployerServo.getSetpoint() == MinerControl.Setpoints.Dump) {
                                UnloadState = 2;
                                LoadTimerStart = Miner.runtime.seconds();
                            }
                            break;
                        case 1:
                            LoadTimerNow = Miner.runtime.seconds() - LoadTimerStart;
                            if (LoadTimerNow > 1.5) {
                                UnloadState = 2;
                                LoadTimerStart = Miner.runtime.seconds();
                            }
                            break;
                        case 2:
                            Miner.DeployerServo.setSetpoint(MinerControl.Setpoints.Undump);
                            UnloadState = 3;
                            LoadTimerStart = Miner.runtime.seconds();
                            break;
                        case 3:
                            LoadTimerNow = Miner.runtime.seconds() - LoadTimerStart;
                            if (LoadTimerNow > 0.5) {
                                UnloadState = 4;
                                LoadTimerStart = Miner.runtime.seconds();
                            }
                            break;
                        case 4:
                            if (!Lift.LifterButtonB.isPressed()) {
                                Lift.Retract();
                            } else {
                                Lift.Stop();
                                UnloadState = 5;
                                LoadTimerStart = Miner.runtime.seconds();
                            }
                            break;
                        case 5:
                            LoadTimerNow = Miner.runtime.seconds() - LoadTimerStart;
                            if (LoadTimerNow > 5.0) {
                                UnloadState = 6;
                                LoadTimerStart = Miner.runtime.seconds();
                            }
                            break;
                        case 6:
                            if (!Miner.MinerButtonI.isPressed()) {
                                Miner.Retract();
                            } else {
                                Miner.MinerStop();
                                UnloadState++;
                            }
                            break;
                        default:
                            Manual = 0;
                            break;
                    }
                    break;

                default:
                    Manual = 0;
                    break;
            }


            telemetry.addLine("DriveSpeed: " + DriveSpeed);
            telemetry.addLine("TurnSpeed: " + TurnSpeed);

            //SubAssembly.test();
            telemetry.update();

            //let the robot have a little rest, sleep is healthy
            sleep(40);
        }
    }
}