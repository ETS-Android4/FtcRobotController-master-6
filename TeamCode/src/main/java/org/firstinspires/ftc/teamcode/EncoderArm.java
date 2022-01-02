package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import static java.lang.Thread.sleep;

public class EncoderArm {
  //initializing the variables that the methods will need later on
  private DcMotor ArmMotor;
  private PIDController pidRotate;
  private double damp = 0.5;
  private double distance_shorten = 2.6;

  public EncoderArm(DcMotor m_ArmMotor){
    //defining the motors as the motor values that we get from the class
    this.ArmMotor = m_ArmMotor;


    //Run using encoders
    ArmMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

    ArmMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

    //setting the motor directions
    ArmMotor.setDirection(DcMotor.Direction.REVERSE);
  }

  public void TurnArm(double power) {
    power = damp * power;

    ArmMotor.setPower(power);
  }

  //Drive forward using encoders
  public void TurnArmDistance(double power, double distance)  {
    distance /= distance_shorten;

    power = damp * power;

    ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

    //Diameter of wheel = 4in.  Circumference = 12.57; Ticks per revolution of goBilda motor = 1136
    //Ticks per inch = 1136/12.57 (approximately 90.37)
    int encoderDistance = (int) (ArmMotor.getCurrentPosition()/2 + distance * 90);

    //Set target position
    ArmMotor.setTargetPosition(encoderDistance);

    //set run to position mode
    ArmMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);


    TurnArm(power);


    while (ArmMotor.isBusy()) {
      TurnArm(power);
    }

    //Stop and change modes back to normal
    StopDriving();
    ArmMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

  }

  public void StopDriving() {

    TurnArm(0);
  }
}