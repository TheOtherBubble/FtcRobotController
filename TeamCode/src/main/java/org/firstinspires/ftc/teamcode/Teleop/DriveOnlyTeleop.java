package org.firstinspires.ftc.teamcode.Teleop;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Hardware;

//import org.firstinspires.ftc.teamcode.src.main.java.org.firstinspires.ftc.teamcode.DriveOnlyHardware;


@TeleOp(name="DriveOnlyTeleop", group="Teleop")

//@Disabled

public class DriveOnlyTeleop extends OpMode {

    Hardware robot = new Hardware();

    private float partialDrive = .4f;
    private float fullDrive = 1f;
    private float drive;

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
        drive = value();
        mecanumMove(drive);
    }

    public void mecanumMove(float driveValue)
    {
        //variables
        double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
        double rightX = gamepad1.right_stick_x;
        final double v1 = r * Math.cos(robotAngle) + rightX;
        final double v2 = r * Math.sin(robotAngle) - rightX;
        final double v3 = r * Math.sin(robotAngle) + rightX;
        final double v4 = r * Math.cos(robotAngle) - rightX;

        robot.fLMotor.setPower(-driveValue * v1);
        robot.fRMotor.setPower(-driveValue * v2);
        robot.bLMotor.setPower(-driveValue * v3);
        robot.bRMotor.setPower(-driveValue * v4);
    }
    public float value() {
        if (gamepad1.left_bumper) {
            return fullDrive;
        }
        else {
            return partialDrive;
        }
    }
}