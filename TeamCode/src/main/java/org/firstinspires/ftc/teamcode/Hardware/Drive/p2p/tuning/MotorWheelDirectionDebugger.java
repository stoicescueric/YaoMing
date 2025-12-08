package org.firstinspires.ftc.teamcode.Hardware.Drive.p2p.tuning;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Hardware.Drive.p2p.MecanumDrive;
import org.firstinspires.ftc.teamcode.Hardware.Drive.p2p.follower.Drivetrain;


/**

 frontLeft-----------frontRight
 |                   |
 |                   |
 |                   |
 |                   |
 |                   |
 |                   |
 rearLeft-----------rearRight


 This opmode will sequentially set power to ur motors in a zig zag pattern
 while waiting for 1 second between setting the power
 */

@Autonomous(group = "Default Tuners")
public class MotorWheelDirectionDebugger extends OpMode {
    Drivetrain drivetrain; //Change to preferred drivetrain
    ElapsedTime timer;


    @Override
    public void init() {
        drivetrain = new MecanumDrive(hardwareMap);
        drivetrain.setDriveMode(Drivetrain.DriveMode.ROBOT_CENTRIC);
        timer = new ElapsedTime();
    }

    @Override
    public void loop() {
        if(timer.seconds() >= 8) {
            timer.reset();
        }

        drivetrain.setMotorPowers( //fix
                (timer.seconds() <= 1) ? 0.7 : 0,
                (Math.abs(timer.seconds() -2.5) <= 0.5) ? 0.7 : 0,
                (Math.abs(timer.seconds() - 4.5) <= 0.5) ? 0.7 : 0,
                (Math.abs(timer.seconds() - 6.5) <= 0.5) ? 0.7 : 0
        );

        drivetrain.update();
    }
}