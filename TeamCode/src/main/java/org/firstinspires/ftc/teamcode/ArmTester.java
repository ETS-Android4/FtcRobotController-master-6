package org.firstinspires.ftc.teamcode;

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

@Autonomous (name = "Arm Test")
@Disabled
public class ArmTester extends LinearOpMode {

  //initializaing the future variables
  private ElapsedTime runtime = new ElapsedTime();
  DcMotor LFMotor, LBMotor, RFMotor, RBMotor, ArmMotor;
  private Servo turnServo, clawServo;
  EncoderAndPIDDriveTrain drive;
  EncoderArm turn;
  BNO055IMU imu;

  //no. of ticks per one revolution of the yellow jacket motors
  int Ticks_Per_Rev = 1316;

  @Override
  public void runOpMode() {
    // Initialize the hardware variables.
    LFMotor = hardwareMap.get(DcMotor.class, "LF Motor");
    LBMotor = hardwareMap.get(DcMotor.class, "LB Motor");
    RFMotor = hardwareMap.get(DcMotor.class, "RF Motor");
    RBMotor = hardwareMap.get(DcMotor.class, "RB Motor");
    ArmMotor = hardwareMap.get(DcMotor.class, "Arm Motor");
    imu = hardwareMap.get(BNO055IMU.class, "imu");

    clawServo = hardwareMap.get(Servo.class, "Claw Servo");
    turnServo = hardwareMap.get(Servo.class, "Turn Servo");

    LFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    LBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    RFMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    RBMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
    ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    //Wheels on the chassis functions
    drive = new EncoderAndPIDDriveTrain(LFMotor, LBMotor, RFMotor, RBMotor, imu);
    turn = new EncoderArm(ArmMotor);

    //Reverse the right motors to move forward based on their orientation on the robot
    clawServo.setDirection(Servo.Direction.FORWARD);
    turnServo.setDirection(Servo.Direction.FORWARD);

    // Wait for the game to start (driver presses PLAY)
    telemetry.addData("Mode", "Init");
    telemetry.update();
    runtime.reset();
    waitForStart();

    //Running the code
    LFMotor.getCurrentPosition();
    if (opModeIsActive()) {
      //Strafe Left to go over the pipes\

      /* High Level

      turn.TurnArmDistance(2,-132);
      turnServo.setPosition(0.5);
      sleep(300);
      clawServo.setPosition(Servo.MIN_POSITION);
      sleep(5000);
      turnServo.setPosition(Servo.MIN_POSITION);
      clawServo.setPosition(0.6);

      */

      /* Mid Goal

      turnServo.setPosition(Servo.MIN_POSITION);
      clawServo.setPosition(0.6);
      sleep(200);
      turn.TurnArmDistance(2,-152);
      turnServo.setPosition(0.65);
      drive.DriveBackwardDistance(1,5);
      sleep(300);
      clawServo.setPosition(Servo.MIN_POSITION);
      sleep(5000);
      turnServo.setPosition(Servo.MIN_POSITION);
      clawServo.setPosition(0.6);
      drive.DriveForwardDistance(1,3);
      sleep(2000);

      */


      /* Low Goal

      turnServo.setPosition(Servo.MIN_POSITION);
      clawServo.setPosition(0.6);
      sleep(200);
      turn.TurnArmDistance(2,-178);
      turnServo.setPosition(Servo.MAX_POSITION);
      //drive.DriveBackwardDistance(1,5);
      sleep(300);
      clawServo.setPosition(Servo.MIN_POSITION);
      sleep(5000);
      turnServo.setPosition(Servo.MIN_POSITION);
      clawServo.setPosition(0.6);
      //drive.DriveForwardDistance(1,3);
      sleep(2000);

      */

    }
  }
}