/* Copyright (c) 2019 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.List;
import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.tfod.TFObjectDetector;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;

/**
 * This 2020-2021 OpMode illustrates the basics of using the TensorFlow Object Detection API to
 * determine the position of the Freight Frenzy game elements.
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this opmode to the Driver Station OpMode list.
 *
 * IMPORTANT: In order to use this OpMode, you need to obtain your own Vuforia license key as
 * is explained below.
 */
@Autonomous(name = "Blue Left Detection Park")
//
//
//  Disabled
public class BlueLeftDetectionPark extends LinearOpMode {

    //initializaing the future variables
    private ElapsedTime runtime = new ElapsedTime();
    DcMotor LFMotor, LBMotor, RFMotor, RBMotor, carouselMotor, armMotor;
    private double carouselSpeed = 0.70;
    private Servo turnServo, clawServo;
    EncoderAndPIDDriveTrain drive;
    EncoderArm turn;
    BNO055IMU imu;

    //no. of ticks per one revolution of the yellow jacket motors
    int Ticks_Per_Rev = 1316;
    /* Note: This sample uses the all-objects Tensor Flow model (FreightFrenzy_BCDM.tflite), which contains
     * the following 4 detectable objects
     *  0: Ball,
     *  1: Cube,
     *  2: Duck,
     *  3: Marker (duck location tape marker)
     *
     *  Two additional model assets are available which only contain a subset of the objects:
     *  FreightFrenzy_BC.tflite  0: Ball,  1: Cube
     *  FreightFrenzy_DM.tflite  0: Duck,  1: Marker
     */
    private static final String TFOD_MODEL_ASSET = "FreightFrenzy_BCDM.tflite";
    private static final String[] LABELS = {
            "Ball",
            "Cube",
            "Duck",
            "Marker"
    };

    /*
     * IMPORTANT: You need to obtain your own license key to use Vuforia. The string below with which
     * 'parameters.vuforiaLicenseKey' is initialized is for illustration only, and will not function.
     * A Vuforia 'Development' license key, can be obtained free of charge from the Vuforia developer
     * web site at https://developer.vuforia.com/license-manager.
     *
     * Vuforia license keys are always 380 characters long, and look as if they contain mostly
     * random data. As an example, here is a example of a fragment of a valid key:
     *      ... yIgIzTqZ4mWjk9wd3cZO9T1axEqzuhxoGlfOOI2dRzKS4T0hQ8kT ...
     * Once you've obtained a license key, copy the string from the Vuforia web site
     * and paste it in to your code on the next line, between the double quotes.
     */
    private static final String VUFORIA_KEY =
            "Afj40JL/////AAABmWe8dKPD80Z7ny7xHv8pE2MFM23nJQdUc6j6Eeg9GuALKSZliFtIOzscXvTyNsqkfzoxYqyFutpPj2aP2ONwJ/eICy58gKsg0ules51TwlEtcSjBnPw9rB6vh1FMkdNml8u76I0H5No3MwjNwF18IrBE1gYzzS1zJanTpF/0gW0F4qQ9m3vLh9gEiPH/F9xZ5zQP6EkCCNy3NWonjYVKrXSkWK8fhja10t7zgqoCFXIrjl9aFeIufrD5KU+FyQ4g+avQCLfx5wyy8ukERZo7OPAGOkxn7U/86gYlt/lrUlsoCxln8jLOwslMpQIxujcrZ9OA+02YqX8XLBTPdhOTrgvX0autXRDjBFzf+bU85WiA";

    /**
     * {@link #vuforia} is the variable we will use to store our instance of the Vuforia
     * localization engine.
     */
    private VuforiaLocalizer vuforia;

    /**
     * {@link #tfod} is the variable we will use to store our instance of the TensorFlow Object
     * Detection engine.
     */
    private TFObjectDetector tfod;

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
        // The TFObjectDetector uses the camera frames from the VuforiaLocalizer, so we create that
        // first.
        initVuforia();
        initTfod();

        /**
         * Activate TensorFlow Object Detection before we wait for the start command.
         * Do it here so that the Camera Stream window will have the TensorFlow annotations visible.
         **/
        if (tfod != null) {
          tfod.activate();

          // The TensorFlow software will scale the input images from the camera to a lower resolution.
          // This can result in lower detection accuracy at longer distances (> 55cm or 22").
          // If your target is at distance greater than 50 cm (20") you can adjust the magnification value
          // to artificially zoom in to the center of image.  For best results, the "aspectRatio" argument
          // should be set to the value of the images used to create the TensorFlow Object Detection model
          // (typically 16/9).
          tfod.setZoom(1.0, 16.0/9.0);
        }

        /** Wait for the game to begin */
        telemetry.addData(">", "Press Play to start op mode");
        telemetry.update();
        waitForStart();

        if (opModeIsActive()) {
          double left = 1000;
          int position = 0;
          int count = 0;
          while (opModeIsActive()) {
            if (tfod != null) {
              // getUpdatedRecognitions() will return null if no new information is available since
              // the last time that call was made.
              List<Recognition> updatedRecognitions = tfod.getUpdatedRecognitions();
              if (updatedRecognitions != null) {
                telemetry.addData("# Object Detected", updatedRecognitions.size());
                // step through the list of recognitions and display boundary info.
                int i = 0;
                for (Recognition recognition : updatedRecognitions) {
                  telemetry.addData(String.format("label (%d)", i), recognition.getLabel());
                  telemetry.addData(String.format("  left,top (%d)", i), "%.03f , %.03f", recognition.getLeft(), recognition.getTop());
                  telemetry.addData(String.format("  right,bottom (%d)", i), "%.03f , %.03f", recognition.getRight(), recognition.getBottom());
                  if((recognition.getLabel() == "Duck" )|| (recognition.getLabel() == "Ball")) {
                    left = recognition.getLeft();
                  }
                  i++;
                }
                if((left > 200) && (left < 1000)) {
                  telemetry.addData("Right Position: ", left);
                  position = 2;
                }
                else if(left < 200) {
                  telemetry.addData("Middle Position: ", left);
                  position = 1;
                }

                telemetry.update();
              }
            }
            count++;
            if(count > 100000) {
              break;
            }
          }

          if(position == 0) {
            telemetry.addData("Left Position: ", position);
          }
          else if(position == 1) {
            telemetry.addData("Middle Position: ", position);
          }

          else if(position == 2) {
            telemetry.addData("Right Position: ", position);
          }

          telemetry.update();

          drive.DriveBackwardDistance(1,2);
          drive.StrafeLeftDistance(1,22);
          drive.DriveForwardDistance(1,4);
          if(position == 2) {
            drive.DriveBackwardDistance(1,15);
            turnServo.setPosition(Servo.MIN_POSITION);
            clawServo.setPosition(0.6);
            turn.TurnArmDistance(2,-132);
            turnServo.setPosition(0.5);
            sleep(300);
            clawServo.setPosition(Servo.MIN_POSITION);
            sleep(500);
            turnServo.setPosition(Servo.MIN_POSITION);
            clawServo.setPosition(0.6);
            drive.DriveForwardDistance(1,10);
            drive.TurnLeftDistance(1,17);
            drive.StrafeRightDistance(1,10);
            turn.TurnArmDistance(2,122);
          }
          else if(position == 1) {
            drive.DriveBackwardDistance(1,4);
            clawServo.setPosition(0.6);
            turnServo.setPosition(Servo.MIN_POSITION);
            turn.TurnArmDistance(2,-156);
            turnServo.setPosition(0.65);
            drive.DriveBackwardDistance(1,5);
            sleep(300);
            clawServo.setPosition(Servo.MIN_POSITION);
            sleep(500);
            clawServo.setPosition(0.6);
            drive.DriveForwardDistance(1,3);
            drive.TurnLeftDistance(1,15);
            drive.StrafeRightDistance(1,15);
            turnServo.setPosition(Servo.MIN_POSITION);
            turn.TurnArmDistance(2,146);
            drive.DriveForwardDistance(1,2);
          }
          else if(position == 0) {
            clawServo.setPosition(0.6);
            turnServo.setPosition(Servo.MIN_POSITION);
            drive.DriveBackwardDistance(1,3);
            turn.TurnArmDistance(2,-178);
            turnServo.setPosition(Servo.MAX_POSITION);
            sleep(100);
            drive.DriveBackwardDistance(1,5);
            clawServo.setPosition(Servo.MIN_POSITION);
            sleep(500);
            clawServo.setPosition(0.6);
            drive.DriveForwardDistance(1,3);
            drive.TurnLeftDistance(1,15);
            drive.StrafeRightDistance(1,13);
            turnServo.setPosition(Servo.MIN_POSITION);
            turn.TurnArmDistance(2, 168);
          }

          drive.DriveBackwardDistance(1,47);


        }
    }

    /**
     * Initialize the Vuforia localization engine.
     */
    private void initVuforia() {
      /*
       * Configure Vuforia by creating a Parameter object, and passing it to the Vuforia engine.
       */
      VuforiaLocalizer.Parameters parameters = new VuforiaLocalizer.Parameters();

      parameters.vuforiaLicenseKey = VUFORIA_KEY;
      parameters.cameraName = hardwareMap.get(WebcamName.class, "duckCam");

      //  Instantiate the Vuforia engine
      vuforia = ClassFactory.getInstance().createVuforia(parameters);

      // Loading trackables is not necessary for the TensorFlow Object Detection engine.
    }

    /**
     * Initialize the TensorFlow Object Detection engine.
     */
    private void initTfod() {
      int tfodMonitorViewId = hardwareMap.appContext.getResources().getIdentifier(
              "tfodMonitorViewId", "id", hardwareMap.appContext.getPackageName());
      TFObjectDetector.Parameters tfodParameters = new TFObjectDetector.Parameters(tfodMonitorViewId);
      tfodParameters.minResultConfidence = 0.8f;
      tfodParameters.isModelTensorFlow2 = true;
      tfodParameters.inputSize = 320;
      tfod = ClassFactory.getInstance().createTFObjectDetector(tfodParameters, vuforia);
      tfod.loadModelFromAsset(TFOD_MODEL_ASSET, LABELS);
    }
}