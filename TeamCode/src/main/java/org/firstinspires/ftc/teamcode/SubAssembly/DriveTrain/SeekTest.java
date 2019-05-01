package org.firstinspires.ftc.teamcode.SubAssembly.DriveTrain;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.teamcode.SubAssembly.Sensors.TofControl;
import org.firstinspires.ftc.teamcode.Utilities.GamepadWrapper;

/* Sub Assembly Test OpMode
 * This TeleOp OpMode is used to test the functionality of the specific sub assembly
 */
// Assign OpMode type (TeleOp or Autonomous), name, and grouping
@TeleOp(name = "Seek Test", group = "Test")
public class SeekTest extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {

        telemetry.addLine("Seek Test: ");

        VoltageSensor vs = hardwareMap.voltageSensor.get("Expansion Hub 2");
        double Voltage = vs.getVoltage();
        telemetry.addData("Voltage", Voltage);
        double Speed = (12.7 - Voltage + 9) / 9;

        /* initialize sub-assemblies
         */
        DriveControl Drive = new DriveControl();
        TofControl Tof = new TofControl();

        GamepadWrapper egamepad1 = new GamepadWrapper(gamepad1);
        GamepadWrapper egamepad2 = new GamepadWrapper(gamepad2);

        Drive.init(this);
        Tof.init(this);
        telemetry.update();

        //waits for that giant PLAY button to be pressed on RC
        waitForStart();

        telemetry.setAutoClear(false);

        while (opModeIsActive()) {
            Drive.forwardUntilDistance4Time(Speed, 15, 8);
            Drive.TimeDelay(5.0);
        }
    }
}