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
        gyroDrive(15, 0, gyroDriveSpeed, 1000);

        pathComplete(500);
    }
}