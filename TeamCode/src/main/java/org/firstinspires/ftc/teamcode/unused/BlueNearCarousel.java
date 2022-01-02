package org.firstinspires.ftc.teamcode.unused;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.EncoderAndPIDDriveTrain;

//Back up Auton that goes to the wall side of the bridge, and parks there

@Autonomous(name = "BLueNearCarousel")
@Disabled
public class BlueNearCarousel extends LinearOpMode {

    //initializaing the future variables
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor LFMotor, LBMotor, RFMotor, RBMotor, carouselMotor;
    private double carouselSpeed = 0.70;
    EncoderAndPIDDriveTrain drive;
    BNO055IMU imu;

    //number of ticks per one revolution of the yellow jacket motors
    int Ticks_Per_Rev = 1316;

    @Override
    public void runOpMode() {
        // Initialize the hardware variables.
        LFMotor = hardwareMap.get(DcMotor.class, "LF Motor");
        LBMotor = hardwareMap.get(DcMotor.class, "LB Motor");
        RFMotor = hardwareMap.get(DcMotor.class, "RF Motor");
        RBMotor = hardwareMap.get(DcMotor.class, "RB Motor");
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        carouselMotor = hardwareMap.get(DcMotor.class, "Carousel Motor");

        LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        //Wheels on the chassis functions
        drive = new EncoderAndPIDDriveTrain(LFMotor, LBMotor, RFMotor, RBMotor, imu);

        //Reverse the right motors to move forward based on their orientation on the robot
        carouselMotor.setDirection(DcMotor.Direction.REVERSE);


        // Wait for the game to start (driver presses PLAY)
        telemetry.addData("Mode", "Init");
        telemetry.update();
        runtime.reset();
        waitForStart();

        //Running the code
        LFMotor.getCurrentPosition();
        if (opModeIsActive()) {
        //Strafe Left to go over the pipes
        drive.StrafeLeftDistance(1,6);
        drive.DriveForwardDistance(1, 55);
        drive.TurnRightDistance(1,7);
        drive.DriveForwardDistance(1, 15);
        drive.DriveForward(0.3);
        carouselMotor.setPower(carouselSpeed);
        sleep(3000);
        drive.DriveForward(0);
        carouselMotor.setPower(0);
        drive.StrafeLeftDistance(1, 12);
        drive.TurnLeftDistance(1,5);
        drive.DriveBackwardDistance(1,10);
        drive.TurnLeftDistance(1, 45);
        drive.DriveBackwardDistance(1,20);
        drive.DriveForwardDistance(1,105);
        drive.StrafeRightDistance(1, 7);
        }
    }
}