package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.SubAssembly.DriveTrain.DriveControl;

import org.firstinspires.ftc.teamcode.SubAssembly.Leds.LedControl;
import org.firstinspires.ftc.teamcode.SubAssembly.Lift.LiftControl;
import org.firstinspires.ftc.teamcode.SubAssembly.Miner.MinerControl;
import org.firstinspires.ftc.teamcode.SubAssembly.Sensors.TofControl;
import org.firstinspires.ftc.teamcode.SubAssembly.Vucam.VucamControl;
import org.firstinspires.ftc.teamcode.Utilities.GamepadWrapper;

/* Sub Assembly Test OpMode
 * This TeleOp OpMode is used to test the functionality of the specific sub assembly
 */
// Assign OpMode type (TeleOp or Autonomous), name, and grouping
@TeleOp(name = "teleop", group = "Drive")
public class teleop extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.setAutoClear(false);
        telemetry.addLine("TeleOp");

        double DriveSpeed = 1;
        double TurnSpeed = DriveSpeed / 2;
        int reverse = 1;
        int Manual = 0;
        int LoadState = 0;
        int UnloadState = 0;
        double TimerStart = 0, TimerNow;

        boolean SILVER = true;
        boolean Slow = false;
        double PrevDriveSpeed = 0;
        double PrevTurnSpeed = 0;

        /* initialize sub-assemblies
         */
        DriveControl Drive = new DriveControl();
        LiftControl Lift = new LiftControl();
        TofControl Tof = new TofControl();
        LedControl Led = new LedControl();
        MinerControl Miner = new MinerControl();
        VucamControl Vucam = new VucamControl();

        GamepadWrapper egamepad1 = new GamepadWrapper(gamepad1);
        GamepadWrapper egamepad2 = new GamepadWrapper(gamepad2);

        Drive.init(this);
        Lift.init(this);
        Tof.init(this);
        Led.init(this);
        Miner.init(this);
        Vucam.init(this);

        //time based variables
        ElapsedTime runtime = new ElapsedTime();
        double lastReset = 0;
        double timeIn = 0;
        double now = 0;

        //waits for that giant PLAY button to be pressed on RC
        telemetry.addLine(">> Press PLAY to start");
        telemetry.update();
        telemetry.setAutoClear(true);
        waitForStart();

        //telling the code to run until you press that giant STOP button on RC
        while (opModeIsActive()) {

            now = runtime.seconds() - lastReset;

            egamepad1.updateEdge();
            egamepad2.updateEdge();

            //Ready Player One

            //reverse control
            if (egamepad1.b.released) {
                reverse = reverse * -1;
            }

            //latch speed setting
            if (egamepad1.a.released) {
                if (Slow) {
                    Slow = false;
                    DriveSpeed = PrevDriveSpeed;
                    TurnSpeed = PrevTurnSpeed;
                } else {
                    Slow = true;
                    PrevDriveSpeed = DriveSpeed;
                    PrevTurnSpeed = TurnSpeed;
                    DriveSpeed = 0.35;
                    TurnSpeed = 0.3;
                }
            }

            //drive speed control
            if (egamepad1.right_bumper.pressed) {
                DriveSpeed += 0.25;
                if (DriveSpeed > 3) DriveSpeed = 3;
            }
            if (egamepad1.right_trigger.pressed) {
                DriveSpeed -= 0.25;
                if (DriveSpeed <= 0) DriveSpeed = 0.25;
            }

            //turning speed control
            if (egamepad1.left_bumper.pressed) {
                TurnSpeed += 0.25;
                if (TurnSpeed > 3) TurnSpeed = 3;
            }
            if (egamepad1.left_trigger.pressed) {
                TurnSpeed -= 0.25;
                if (TurnSpeed <= 0) TurnSpeed = 0.25;
            }

            //drive controls
            if (-gamepad1.left_stick_y < -0.4) {
                Drive.moveBackward(reverse * DriveSpeed);
            } else if (-gamepad1.left_stick_y > 0.4) {
                Drive.moveForward(reverse * DriveSpeed);
            } else if (gamepad1.left_stick_x > 0.4) {
                Drive.turnRight(TurnSpeed);
            } else if (gamepad1.left_stick_x < -0.4) {
                Drive.turnLeft(TurnSpeed);
            } else {
                Drive.stop();
            }

            //Ready Player Two

            if (egamepad2.x.released) {
                SILVER = true;
            } else if (egamepad2.y.released) {
                SILVER = false;
            }

            if ((gamepad2.left_stick_y < -0.4) || (gamepad2.left_stick_y > 0.4) || (egamepad2.dpad_up.pressed) || (egamepad2.dpad_down.pressed) || (egamepad2.left_bumper.pressed) || (egamepad2.left_trigger.pressed)) {
                Manual = 0;
            } else if (egamepad2.a.pressed) {
                Manual = 1;
            } else {
                /* check if read to load or unload */
                if (Lift.LifterButtonB.isPressed()) {
                    telemetry.addLine("ready to load");
                    if (egamepad2.b.pressed) {
                        Manual = 2;
                        LoadState = 0;
                    }
                }
                if (Miner.DeployerServo.getSetpoint() != MinerControl.Setpoints.Undump) {
                    telemetry.addLine("ready to unload");
                    if (egamepad2.b.pressed) {
                        Manual = 3;
                        UnloadState = 0;
                    }
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
                    } else if (gamepad2.right_stick_x > 0.4) {
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
                                TimerStart = Miner.runtime.seconds();
                            }
                            break;
                        case 1:
                            Miner.Intake();
                            TimerNow = Miner.runtime.seconds() - TimerStart;
                            if (SILVER) {
                                Miner.IntakeRaise();
                                Led.white();
                                if (TimerNow > 0.8) {
                                    LoadState++;
                                    TimerStart = Miner.runtime.seconds();
                                }
                            } else if (!SILVER) {
                                Miner.IntakeLower();
                                Led.orange();
                                if (TimerNow > 1.2) {
                                    LoadState++;
                                    TimerStart = Miner.runtime.seconds();
                                }
                            }
                            break;
                        case 2:
                            Miner.Stoptake();
                            Miner.Extend();
                            if (SILVER) {
                                Miner.IntakeRaise();
                            } else if (!SILVER) {
                                Miner.IntakeLower();
                            }
                            TimerNow = Miner.runtime.seconds() - TimerStart;
                            if (TimerNow == 0.25) {
                                Miner.DeployerServo.setSetpoint(MinerControl.Setpoints.Middump);
                            } else if (TimerNow > 0.5) {
                                Miner.MinerStop();
                                Miner.DeployerServo.setSetpoint(MinerControl.Setpoints.Middump);
                                LoadState++;
                                TimerStart = Miner.runtime.seconds();
                            }
                            break;
                        case 3:
                            Lift.Extend();
                            TimerNow = Miner.runtime.seconds() - TimerStart;
                            if (TimerNow > 1.5) {
                                Lift.Stop();
                                LoadState++;
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
                                TimerStart = Miner.runtime.seconds();
                            } else if (Miner.DeployerServo.getSetpoint() == MinerControl.Setpoints.Dump) {
                                UnloadState = 2;
                                TimerStart = Miner.runtime.seconds();
                            }
                            break;
                        case 1:
                            TimerNow = Miner.runtime.seconds() - TimerStart;
                            if (TimerNow > 1.5) {
                                UnloadState++;
                                TimerStart = Miner.runtime.seconds();
                            }
                            break;
                        case 2:
                            Miner.DeployerServo.setSetpoint(MinerControl.Setpoints.Undump);
                            UnloadState++;
                            TimerStart = Miner.runtime.seconds();
                            break;
                        case 3:
                            TimerNow = Miner.runtime.seconds() - TimerStart;
                            if (TimerNow > 0.5) {
                                UnloadState++;
                                TimerStart = Miner.runtime.seconds();
                            }
                            break;
                        case 4:
                            TimerNow = Miner.runtime.seconds() - TimerStart;
                            if (Miner.MinerButtonI.isPressed()) {
                                if (TimerNow < 0.5) {
                                    Miner.Extend();
                                } else {
                                    Miner.MinerStop();
                                    UnloadState++;
                                    TimerStart = Miner.runtime.seconds();
                                }
                            } else {
                                UnloadState++;
                                TimerStart = Miner.runtime.seconds();
                            }
                            break;
                        case 5:
                            if (!Lift.LifterButtonB.isPressed()) {
                                Lift.Retract();
                            } else {
                                Lift.Stop();
                                UnloadState++;
                                TimerStart = Miner.runtime.seconds();
                            }
                            break;
                        case 6:
                            TimerNow = Miner.runtime.seconds() - TimerStart;
                            if (SILVER) {
                                Miner.IntakeRaise();
                            } else if (!SILVER) {
                                Miner.IntakeLower();
                            }
                            if (TimerNow > 0.5) {
                                UnloadState++;
                                TimerStart = Miner.runtime.seconds();
                            }
                            break;
                        case 7:
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