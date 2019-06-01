package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.SubAssembly.DriveTrain.DriveUntilColorTest;
import org.firstinspires.ftc.teamcode.SubAssembly.Miner.MinerControl;
import org.firstinspires.ftc.teamcode.SubAssembly.Sensors.IMUcontrol;
import org.firstinspires.ftc.teamcode.SubAssembly.Sensors.TofControl;
import org.firstinspires.ftc.teamcode.SubAssembly.Vucam.VucamControl;
import org.firstinspires.ftc.teamcode.Utilities.UserControl;
import org.firstinspires.ftc.teamcode.SubAssembly.DriveTrain.DriveControl;
import org.firstinspires.ftc.teamcode.SubAssembly.Lift.LiftControl;
import org.firstinspires.ftc.teamcode.SubAssembly.Claimer.ClaimerControl;
import org.firstinspires.ftc.teamcode.Utilities.AutoTransitioner;
import org.firstinspires.ftc.teamcode.SubAssembly.Leds.LedControl;
import org.firstinspires.ftc.teamcode.SubAssembly.Sensors.ColorControl;

@Autonomous(name = "Do Not Select!!!", group = "Sentience")
public class ultronPrime extends LinearOpMode {

    /* Sub assemblies */
    UserControl User = new UserControl();
    VucamControl Vucam = new VucamControl();
    DriveControl Drive = new DriveControl();
    LedControl Led = new LedControl();
    TofControl Tof = new TofControl();
    IMUcontrol Imu = new IMUcontrol();

    //time based variables
    private ElapsedTime runtime = new ElapsedTime();
    private double lastReset = 0;
    private double now = 0;
    private double distThresh = 60;
    double origDist = 0;
    double bestDist = 0;
    double bestAngle = 0;
    int runs = 0;

    /* Constants (set at start of match) */
    double iSpeed = 1.0;


    public void resetClock() {
        //resets the clock
        lastReset = runtime.seconds();
    }

    /* Variables go here */


    //Enum variables (creates variables of the enum variable types previously created)
    private int timeDelay = 0;


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
        Led.init(this);
        Tof.init(this);
        Imu.init(this);


        //waits for that giant PLAY button to be pressed on RC
        telemetry.addLine(">> Press PLAY to start");
        telemetry.update();
        telemetry.setAutoClear(true);
        waitForStart();

        telemetry.update();
        telemetry.setAutoClear(false);

        Drive.imu.setStartAngle();

        while (opModeIsActive()) {
            now = runtime.seconds() - lastReset;
            if (Tof.getDistance3() > distThresh) {
                Led.red();
                sleep(500);
                Drive.moveForward(0.3);
            } else {
                while (runs < 5) {
                    Drive.stop();
                    Led.orange();
                    sleep(500);
                    origDist = Tof.getDistance3();
                    Drive.turnAngle(0.6, 90);
                    if (Tof.getDistance3() > origDist) {
                        Led.lawnGreen();
                        sleep(500);
                        bestDist = Tof.getDistance3();
                        bestAngle = Imu.currentAngle;
                        telemetry.addLine("Distance: " + bestDist);
                        telemetry.addLine("Angle: " + bestAngle);
                    } else {
                        Led.red();
                        sleep(500);
                    }
                    runs++;
                }
                break;
                /*
                Drive.stop();
                Led.orange();
                sleep(500);
                Drive.turnAngle(0.5, 90);
                if (Tof.getDistance3() > bestDist) {
                    Led.lawnGreen();
                    sleep(500);
                    bestDist = Tof.getDistance3();
                    bestAngle = Imu.currentAngle;
                    telemetry.addLine("Distance: " +bestDist);
                    telemetry.addLine("Angle: " +bestAngle);
                } else {
                    Led.red();
                    sleep(500);
                }
                Drive.stop();
                Led.orange();
                sleep(500);
                Drive.turnAngle(0.5, 90);
                if (Tof.getDistance3() > bestDist) {
                    Led.lawnGreen();
                    sleep(500);
                    bestDist = Tof.getDistance3();
                    bestAngle = Imu.currentAngle;
                    telemetry.addLine("Distance: " +bestDist);
                    telemetry.addLine("Angle: " +bestAngle);
                } else {
                    Led.red();
                    sleep(500);
                }
                Drive.stop();
                Led.orange();
                sleep(500);
                Drive.turnAngle(0.5, 90);
                if (Tof.getDistance3() > bestDist) {
                    Led.lawnGreen();
                    sleep(500);
                    bestDist = Tof.getDistance3();
                    bestAngle = Imu.currentAngle;
                    telemetry.addLine("Distance: " +bestDist);
                    telemetry.addLine("Angle: " +bestAngle);
                } else {
                    Led.red();
                    sleep(500);
                }
                break;
                */
            }

                Drive.imu.update();

                Drive.Tof.Telemetry();

                sleep(40);
            }

            Drive.stop();

            telemetry.setAutoClear(true);
        }
    }