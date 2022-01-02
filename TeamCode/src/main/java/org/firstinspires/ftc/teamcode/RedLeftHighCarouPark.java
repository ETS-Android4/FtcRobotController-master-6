package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

//Back up Auton that goes to the wall side of the bridge, and parks there

@Autonomous (name = "Red Left High Carou Park")
//@Disabled
public class RedLeftHighCarouPark extends LinearOpMode {

  //initializaing the future variables
  private ElapsedTime runtime = new ElapsedTime();
  DcMotor LFMotor, LBMotor, RFMotor, RBMotor, carouselMotor, armMotor;
  private double carouselSpeed = 0.70, armSpeed=0.6;
  private Servo turnServo, clawServo;
  EncoderAndPIDDriveTrain drive;
  EncoderArm turn;
  BNO055IMU imu;

  //no. of ticks per one revolution of the yellow jacket motors
  int Ticks_Per_Rev = 1316;

  @Override
  public void runOpMode() throws InterruptedException {
    // Initialize the hardware variables.
    LFMotor = hardwareMap.get(DcMotor.class, "LF Motor");
    LBMotor = hardwareMap.get(DcMotor.class, "LB Motor");
    RFMotor = hardwareMap.get(DcMotor.class, "RF Motor");
    RBMotor = hardwareMap.get(DcMotor.class, "RB Motor");
    imu = hardwareMap.get(BNO055IMU.class, "imu");
    carouselMotor = hardwareMap.get(DcMotor.class, "Carousel Motor");
    armMotor = hardwareMap.get(DcMotor.class, "Arm Motor");

    clawServo = hardwareMap.get(Servo.class, "Claw Servo");
    turnServo = hardwareMap.get(Servo.class, "Turn Servo");

    LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    armMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    //Wheels on the chassis functions
    drive = new EncoderAndPIDDriveTrain(LFMotor, LBMotor, RFMotor, RBMotor, imu);
    turn = new EncoderArm(armMotor);

    //Reverse the right motors to move forward based on their orientation on the robot
    carouselMotor.setDirection(DcMotor.Direction.REVERSE);

    clawServo.setDirection(Servo.Direction.FORWARD);
    turnServo.setDirection(Servo.Direction.FORWARD);

    carouselMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);



    // Wait for the game to start (driver presses PLAY)
    telemetry.addData("Mode", "Init");
    telemetry.update();
    runtime.reset();
    waitForStart();


    //Running the code
    LFMotor.getCurrentPosition();
    if (opModeIsActive()) {
      drive.StrafeLeftDistance(1,28);
      drive.DriveForwardDistance(1,1);
      drive.DriveBackwardDistance(1,14);
      turnServo.setPosition(Servo.MIN_POSITION);
      clawServo.setPosition(0.6);
      turn.TurnArmDistance(2,-132);
      turnServo.setPosition(0.5);
      sleep(300);
      clawServo.setPosition(Servo.MIN_POSITION);
      sleep(500);
      turnServo.setPosition(Servo.MIN_POSITION);
      clawServo.setPosition(0.6);
      drive.DriveForwardDistance(1,14);
      turn.TurnArmDistance(2,122);

      drive.StrafeRightDistance(1,44);
      drive.StrafeRightDistance(0.5,3);
      carouselMotor.setPower(-carouselSpeed);
      sleep(3000);
      carouselMotor.setPower(0);

      drive.DriveBackwardDistance(1,18);
      drive.StrafeRightDistance(1,10);
      drive.DriveBackwardDistance(1,6);

    }
  }


}