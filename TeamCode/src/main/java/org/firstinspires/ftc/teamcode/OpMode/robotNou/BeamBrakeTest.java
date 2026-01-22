package org.firstinspires.ftc.teamcode.OpMode.robotNou;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DigitalChannel;

import org.firstinspires.ftc.teamcode.Util.Wrapper.DigitalWrapper;

@Config
@TeleOp(name = "Beam brake test")
public class BeamBrakeTest extends LinearOpMode {
    public static String beamBrakeName = "beamBrakePos1";
    DigitalChannel rawBeamBrake;

    DigitalWrapper sensor;
    boolean currentValue = false;
    @Override
    public void runOpMode() throws InterruptedException {
        sensor = new DigitalWrapper(hardwareMap, beamBrakeName);
        rawBeamBrake = hardwareMap.get(DigitalChannel.class, beamBrakeName);
        rawBeamBrake.setMode(DigitalChannel.Mode.INPUT);
        waitForStart();
        while (opModeIsActive()){
            currentValue = sensor.getValue();
            telemetry.addData("Beam brake Wrapper state", currentValue);
            telemetry.addData("Beam brake Wrapper state raw", sensor.getRaw());

            telemetry.addData("Beam brake raw state", rawBeamBrake.getState());
            telemetry.addData("Beam brake raw state raw", !rawBeamBrake.getState());

            telemetry.update();
        }
    }
}
