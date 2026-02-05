package org.firstinspires.ftc.teamcode.OpMode.Tuning;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;


@Config
@TeleOp(name = "Test localizare")
public class LaserSensor extends LinearOpMode {

    public static String name = "laser";
    public static String name2 = "laser2";
    public static final double MAX_VOLTS = 3.3;
    public static final double MAX_DISTANCE_MM = 5000.0;
    AnalogInput input;
    AnalogInput input2;
    @Override
    public void runOpMode() throws InterruptedException {
        input = hardwareMap.get(AnalogInput.class, name);
        input2 = hardwareMap.get(AnalogInput.class, name2);
        waitForStart();
        while (opModeIsActive()) {
            double voltage1 = input.getVoltage();
            double voltage2 = input2.getVoltage();
            double distanceMM_1 = (voltage1 / MAX_VOLTS) * MAX_DISTANCE_MM;
            double distanceMM_2 = (voltage2 / MAX_VOLTS) * MAX_DISTANCE_MM;

            telemetry.addData("Voltage 1", voltage1);
            telemetry.addData("Distance 1 mm", distanceMM_1);
            telemetry.addData("Distance 1 inches", distanceMM_1 / 25.4);

            telemetry.addData("Voltage 1", voltage2);
            telemetry.addData("Distance 1 mm", distanceMM_2);
            telemetry.addData("Distance 1 inches", distanceMM_2 / 25.4);


            telemetry.addData("Details 1", input.getDeviceName() + "  " + input.getConnectionInfo());
            telemetry.addData("Details 2", input2.getDeviceName() + "  " + input2.getConnectionInfo());

            telemetry.update();
        }
    }
}
