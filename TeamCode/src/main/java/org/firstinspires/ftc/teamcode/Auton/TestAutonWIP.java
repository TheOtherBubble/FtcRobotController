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
        //turnToPosition(90, xyz, turnSpeed, 10, false);
        //turnToPosition(-90, xyz, turnSpeed, 10, false);
        //turnToPosition(-179, xyz, turnSpeed, 10, false);
        gyroDrive(20, 'l', readAngle(xyz), gyroDriveSpeedStrafe, 500);
        //strafe(1, strafeSpeed, right, rightBalRed, 600, rightMoreBal, NORTH); //strafe template
        //works with 200ms, 300ms, 500ms, does not work at 800ms other vals have not been tested
        //strafe(1, strafeSpeed, left, leftBalRed, 600, leftMoreBal, NORTH); //ending turntoposition is sus

        pathComplete(500);
    }
}