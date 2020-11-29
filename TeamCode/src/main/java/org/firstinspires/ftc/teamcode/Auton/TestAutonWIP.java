package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.AutonDrivingWIP;

@Autonomous(name="TestAutonWIP", group="GyroAuton")
public class TestAutonWIP extends AutonDrivingWIP {
//TESTING CLASS DO NOT USE
    @Override
    public void runOpMode() {

        //init
        robot.init(hardwareMap);
        initImu();
        //initVuforia(); //may not work unsure
        stopAndReset();

        waitForStart();

        sleep(500);
        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        //turnDegrees(-90, xyz, turnSpeed, 5000);
        //turnDegrees(180, xyz, turnSpeed, 5000);
        turnToPosition(90, xyz, turnSpeed, 10, false);
        turnToPosition(-90, xyz, turnSpeed, 10, false);
        turnToPosition(-179, xyz, turnSpeed, 10, false);
        //gyroDrive(26.5, readAngle(xyz), gyroDriveSpeed, 1000);

        pathComplete(500);
    }
}