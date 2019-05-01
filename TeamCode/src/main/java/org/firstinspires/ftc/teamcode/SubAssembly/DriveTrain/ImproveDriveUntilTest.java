package org.firstinspires.ftc.teamcode.SubAssembly.DriveTrain;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.SubAssembly.Claimer.ClaimerControl;
import org.firstinspires.ftc.teamcode.SubAssembly.Sensors.ColorControl;
import org.firstinspires.ftc.teamcode.SubAssembly.Sensors.IMUcontrol;
import org.firstinspires.ftc.teamcode.SubAssembly.Sensors.TofControl;
import org.firstinspires.ftc.teamcode.Utilities.GamepadWrapper;

/* Sub Assembly Test OpMode
 * This TeleOp OpMode is used to test the functionality of the specific sub assembly
 */
// Assign OpMode type (TeleOp or Autonomous), name, and grouping
@TeleOp(name = "Improve Drive Until Test", group = "Test")
public class ImproveDriveUntilTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("Drive Until Color Test: ");
        double speed = 0.50;
        double time = 0.125;
        boolean move2claim = false;

        /* initialize sub-assemblies
         */
        DriveControl Drive = new DriveControl();
        TofControl Tof = new TofControl();
        IMUcontrol Imu = new IMUcontrol();
        ColorControl Color = new ColorControl();
        ClaimerControl Claimer = new ClaimerControl();

        GamepadWrapper egamepad1 = new GamepadWrapper(gamepad1);
        GamepadWrapper egamepad2 = new GamepadWrapper(gamepad2);

        Drive.init(this);
        Tof.init(this);
        Claimer.init(this);
        telemetry.update();

        //waits for that giant PLAY button to be pressed on RC
        waitForStart();

        while (opModeIsActive()) {

            egamepad1.updateEdge();
            egamepad2.updateEdge();

                /*do {
                    Drive.moveForward(speed);
                    if (Tof.getDistance3() < 90) {
                        Drive.DriveUntilColor(0.25);
                        Claimer.drop();
                        move2claim = true;
                    }
                } while (!move2claim);
                */

            if (egamepad1.a.released) {
                while (Tof.getDistance3() > 30) {
                    Drive.moveForward(0.35);
                    if (Tof.getDistance3() < 90) {
                        Drive.DriveUntilColor(0.25,0.20);
                        if (Tof.getDistance3() > 30) {
                            break;
                        }
                    }
                }
                Claimer.drop();
            }

            if (egamepad1.b.released) {
                    Drive.DriveUntilColor(0.25,0.20);
                    Claimer.drop();
            }

            if (egamepad1.dpad_down.state) {
                Drive.moveBackward(speed);
            } else {
                Drive.stop();
            }

            if (egamepad1.b.released) {
                Drive.stop();
            }


            //SubAssembly.test();
            telemetry.update();

            //let the robot have a little rest, sleep is healthy
            sleep(40);
        }
    }
}