package org.firstinspires.ftc.teamcode.Auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import org.firstinspires.ftc.robotcore.external.navigation.Position;
import org.firstinspires.ftc.robotcore.external.navigation.Velocity;
import org.firstinspires.ftc.teamcode.AutonDrivingWIP;

@Autonomous(name="TestAuton", group="GyroAuton")
public class TestAutonWIP extends AutonDrivingWIP {
//TESTING CLASS DO NOT USE
    @Override
    public void runOpMode() {

        //init
        robot.init(hardwareMap);
        initImu();
        initVuforia();
        stopAndReset();

        waitForStart();

        imu.startAccelerationIntegration(new Position(), new Velocity(), 1000);

        vuforia(allTrackables, targetsUltimateGoal, 500);

        pathComplete(500);
    }
}