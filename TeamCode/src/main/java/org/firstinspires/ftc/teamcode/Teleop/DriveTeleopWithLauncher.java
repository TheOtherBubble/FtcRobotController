package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;

//import org.firstinspires.ftc.teamcode.src.main.java.org.firstinspires.ftc.teamcode.DriveOnlyHardware;


@TeleOp(name="Teleop with launcher", group="Teleop")

//@Disabled

public class DriveTeleopWithLauncher extends OpMode {

    Hardware robot = new Hardware();

    private float drive = .6f;
    //private float BRDrive = 1f;

    @Override
    public void init()
    {
        //Initialize the hardware variables.
        //The init() method of the hardware class does all the work here
        robot.init(hardwareMap);


    }

    @Override
    public void loop()

    {
        if(gamepad1.a)
        {
            if (robot.launcherServo.getPosition() < 1) {
                robot.launcherServo.setPosition(robot.launcherServo.getPosition() + 0.01);
            }
        }
        if (gamepad1.b)
        {
            if (robot.launcherServo.getPosition() > 0) {
                robot.launcherServo.setPosition(robot.launcherServo.getPosition() - 0.01);
            }
        }

        if(gamepad1.right_bumper)
        {
            robot.launcherMotor.setPower(1);
        }
        else if(gamepad1.left_bumper)
        {
            robot.launcherMotor.setPower(-1);
        }
        else
        {
            robot.launcherMotor.setPower(0);
        }

        mecanumMove();

    }

    public void mecanumMove()
    {
        //variables
        double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) - rightX;
        final double v2 = r * Math.sin(robotAngle) + rightX;
        final double v3 = r * Math.sin(robotAngle) - rightX;
        final double v4 = r * Math.cos(robotAngle) + rightX;

        robot.fLMotor.setPower(-drive * v1);
        robot.fRMotor.setPower(-drive * v2);
        robot.bLMotor.setPower(-drive * v3);
        robot.bRMotor.setPower(-drive * v4);

        telemetry.addData("fLPower", -drive * v1);
        telemetry.addData("fRPower", -drive * v2);
        telemetry.addData("bLPower", -drive * v3);
        telemetry.addData("bRPower", -drive * v4);
    }
}