package org.firstinspires.ftc.teamcode.SubAssembly.DriveTrain;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Utilities.GamepadWrapper;

/* Sub Assembly Test OpMode
 * This TeleOp OpMode is used to test the functionality of the specific sub assembly
 */
// Assign OpMode type (TeleOp or Autonomous), name, and grouping
@TeleOp(name = "Drive Until Color Test", group = "Test")
public class DriveUntilColorTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("Drive Until Color Test: ");
        double speed = 0.25;
        double time = 0.125;

        /* initialize sub-assemblies
         */
        DriveControl Drive = new DriveControl();

        GamepadWrapper egamepad1 = new GamepadWrapper(gamepad1);
        GamepadWrapper egamepad2 = new GamepadWrapper(gamepad2);

        Drive.init(this);
        telemetry.update();

        //waits for that giant PLAY button to be pressed on RC
        waitForStart();

        while (opModeIsActive()) {

            egamepad1.updateEdge();
            egamepad2.updateEdge();


            if (egamepad1.left_bumper.released) {
                speed -= 0.05;
            } else if (egamepad1.right_bumper.released) {
                speed += 0.10;
            }

            if (egamepad1.right_trigger.released) {
                time += 0.025;
            } else if (egamepad1.left_trigger.released) {
                time -= 0.025;
            }

            if (egamepad1.a.released) {
                Drive.DriveUntilColor(0.25,0.20);
                Drive.moveBackward(0.25, time);
            }

            if (egamepad1.b.released) {
                Drive.DriveUntilColor(0.25,0.20);
                Drive.moveBackward(0.25, time);
            }

            if (egamepad1.dpad_down.pressed) {
                Drive.moveBackward(speed);
            } else {
                Drive.stop();
            }

            if (egamepad1.b.released) {
                Drive.stop();
            }

            telemetry.addLine("time" + time);
            telemetry.addLine("speed " + speed);

            //SubAssembly.test();
            telemetry.update();

            //let the robot have a little rest, sleep is healthy
            sleep(40);
        }
    }
}