package org.firstinspires.ftc.teamcode.tests;//Hollow World

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvPipeline;




//------------------------------------------------------------------------------//
@TeleOp(name="CameraDiagnostics", group="Linear Opmode")

public class CameraDiagnostics extends LinearOpMode {






    double getBatteryVoltage() {
        double result = Double.POSITIVE_INFINITY;
        for (VoltageSensor sensor : hardwareMap.voltageSensor) {
            double voltage = sensor.getVoltage();
            if (voltage > 0) {
                result = Math.min(result, voltage);
            }
        } return result;

    }




    private static DcMotor frontl = null;
    private static DcMotor frontr = null;
    private static DcMotor bottoml = null;
    private static DcMotor bottomr = null;
    private static DcMotor ElevatorMotor = null;
    private static DcMotor CarouselMotor = null;
    private static CRServo servo = null;
    // private static DcMotor shooter = null;
    //   private static CRServo frontservo = null;
    // private DistanceSensor distanceSensor;
//---\
//   private static CRServo lowestServo = null;
    //  private static CRServo middleServo = null;
    // private static CRServo highestServo = null;
    // private static DcMotor ArmMotor = null;
    // private static CRServo ServoArm = null;
    // private static ColorSensor ColorSensorRPM = null;
//-------------------------------------------------------------------------------//
    enum PowerLevel {MAX, HALF, QUARTER, STOP};

    // Declare OpMode members/constants.
    private ElapsedTime runtime = new ElapsedTime();
    //private static final double ADJUST_DIAGONAL = 0.40; ***Not used anymore***
    private static double MOTOR_ADJUST = 0.75;
    //Defining Motor Arm Movement
    private static double SMALL_GEAR_TEETH = 40; //Smaller gear is directly on the motor
    private static double LARGE_GEAR_TEETH = 120; //Larger gear is connected to the arm
    private static double GEAR_RATIO = (LARGE_GEAR_TEETH/SMALL_GEAR_TEETH); //States the gear ratio based on the teeth of gears used
    private static double ORIBITAL_60_PULSE_PER_ROTATION = 1680; //Motor is an AndyMark Neverest 60 GearMotor
    private static double ARM_PULSE_PER_ROTATION = ORIBITAL_60_PULSE_PER_ROTATION * GEAR_RATIO; //Amount of rotation per motor to gears
    private static double DEGREES_PER_TICK = 360/ARM_PULSE_PER_ROTATION; //Converts Radians into Degrees
    //Power for Motor Arm
    private static double MOTOR_ARM_MAX_POWER = .7;
    private static double MOTOR_ARM_MIN_POWER = .3;
    private static double ARM_HOLDING_POWER = .15;
    private static int OPT_PICKUP_HEIGHT = 186;
    private static int OPT_TRAV_HEIGHT = 459;
    private final long BILLION = 1000000000;
    private static double SIDEWAYS_DRIFT_CORRECTION = 1.125;
    private int x = 1;
    private int I = 0;
    private boolean aPressed = false;
    private boolean bPressed = false;
    private boolean xPressed = false;
    private boolean ypressed = false;
    //double initialServoPosition = ServoArm.getPosition();

    OpenCvCamera webcam;
    TripleRectPipeline pipeline;

    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());

        pipeline = new TripleRectPipeline();

        PowerLevel powerLevel = PowerLevel.HALF.QUARTER;
        //double initialServoPosition = ServoArm.getPosition();//Starts the robot wheels at MAX power level
        frontl= hardwareMap.get(DcMotor.class, "frontl");
        frontr= hardwareMap.get(DcMotor.class, "frontr");
        bottoml= hardwareMap.get(DcMotor.class, "bottoml");
        bottomr= hardwareMap.get(DcMotor.class, "bottomr");
        servo = hardwareMap.get(CRServo.class,"servo");
        CarouselMotor = hardwareMap.get(DcMotor.class, "CarouselMotor");
        ElevatorMotor= hardwareMap.get(DcMotor.class, "ElevatorMotor");
        //    shooter= hardwareMap.get(DcMotor.class, "shooter");
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "OpenCvCamera"), cameraMonitorViewId);
        webcam.setPipeline(pipeline);
        frontl.setDirection(DcMotor.Direction.FORWARD);
        frontr.setDirection(DcMotor.Direction.REVERSE);
        bottoml.setDirection(DcMotor.Direction.FORWARD);
        bottomr.setDirection(DcMotor.Direction.REVERSE);
        ElevatorMotor.setDirection(DcMotor.Direction.FORWARD);
        CarouselMotor.setDirection(DcMotor.Direction.REVERSE);





        //Brake immedietly after joystick hits 0 instead of coasting down
        frontl.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        frontr.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        bottoml.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        bottomr.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        ElevatorMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        CarouselMotor.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        //---




        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                //1280 x 720 resolution, aka 720p
                webcam.startStreaming(1280,720, OpenCvCameraRotation.UPRIGHT);

            }

            @Override
            public void onError(int errorCode)
            {

            }
        });



        // Wait for the game to start (driver presses PLAY)
        waitForStart();
//------------------------------------------------------------------ Start of Match ---------------------------------------------------
        runtime.reset();
        frontl.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottoml.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        bottomr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);


        frontl.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottoml.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        bottomr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        // Setup a variable for each drive wheel
        double frontlPower = 0;
        double frontrPower = 0;
        double bottomlPower = 0;
        double bottomrPower = 0;
        double CarouselMotorPower = 0;
        double ArmMotorPower = 0;

        double ElevatorMotorPower = 1;
        double triggerPowerAdjust = 1;

//anyd is baldsm
        while (opModeIsActive()) {  //While Teleop is in session






//          ************************************************ GAMEPAD 2 CONTROLS ************************************************









            //3 touch// 440 =  optimum pickup// 200 = bottom// 400 = optimum travel height/ 1880 drop height// 2958 drop height l



            if (gamepad1.dpad_up) {
                ElevatorMotor.setPower(1);

            }
            else if (gamepad1.dpad_down) {
                ElevatorMotor.setPower(-1);
            }
            else {
                ElevatorMotor.setPower(0);
            }
            if (gamepad1.y) {
                CarouselMotor.setPower(-1);
            }
            else {
                CarouselMotor.setPower(0);
            }

            if (gamepad1.b) {
                servo.setPower(-1);
            }
            else if (gamepad1.a) {
                servo.setPower(1);
            }
            else {
                servo.setPower(0);
            }



            if (gamepad1.right_trigger > 0 ) {
                triggerPowerAdjust = .4;
            }
            else {
                triggerPowerAdjust = 1;
            }



            //See the Desmos for an explanation, this is code that's basically modified from what they (FTC) gave us
            double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
            double robotAngle = Math.atan2(gamepad1.left_stick_y, -gamepad1.left_stick_x) - Math.PI / 4;
            double rightX = -gamepad1.right_stick_x * 0.5;
            double v1 = r * Math.cos(robotAngle) + rightX;
            double v2 = r * Math.sin(robotAngle) - rightX;
            double v3 = r * Math.sin(robotAngle) + rightX;
            double v4 = r * Math.cos(robotAngle) - rightX;

            v1 = v1*MOTOR_ADJUST*triggerPowerAdjust;
            v2 = v2*MOTOR_ADJUST*triggerPowerAdjust;
            v3 = v3*MOTOR_ADJUST*triggerPowerAdjust;
            v4 = v4*MOTOR_ADJUST*triggerPowerAdjust;
            frontl.setPower(v1*1);
            frontr.setPower(v2*1);
            bottoml.setPower(v3*1);
            bottomr.setPower(v4*1);
            //  telemetry.addData("Motor Power", "v1 (%.2f), v2 (%.2f) v3 (%.2f) v4 (%.2f)", v1,v2,v3,v4);
            //  telemetry.addData("R","%.2f",r);
            telemetry.addData("Threshold A", pipeline.getRegionA());
            telemetry.addData("Threshold B", pipeline.getRegionB());
            telemetry.addData("Threshold C", pipeline.getRegionC());


//          ************************************************ GAMEPAD 2 CONTROLS ************************************************


            //Telemetry
            telemetry.update();

        }//While opMode
    }//void runOpMode

    public class TripleRectPipeline extends OpenCvPipeline {
        //Colors Defined
        private final Scalar RED = new Scalar(255,0,0);
        private final Scalar BLUE = new Scalar(0, 0, 255);
        private final Scalar GREEN = new Scalar(0, 255, 0);

        //I just want someone to tickle my balls<> -Noah
        private static final int THRESHOLD = 150;

        /*

        [A]    [B]     [C]
         */
        Point aTopLeft = new Point(70, 260);
        Point aBottomRight = new Point(220, 410);

        Point bTopLeft = new Point(570, 270);
        Point bBottomRight = new Point(700, 390);

        Point cTopLeft = new Point(1050, 290);
        Point cBottomRight = new Point(1180, 415);

        //YCrCb  --> Y = Brightness, Cr = Red - Luma, Cb = Blue - Luma
        Mat regionA_Cb;
        Mat regionB_Cb;
        Mat regionC_Cb;

        Mat YCrCb = new Mat();
        Mat Cb = new Mat();

        private volatile int averageA;
        private volatile int averageB;
        private volatile int averageC;
        //private string type = "N/a";
        private void inputToCb(Mat input) {
            Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
            Core.extractChannel(YCrCb, Cb, 2);
        }

        @Override
        public void init(Mat input) {
            inputToCb(input);

            regionA_Cb = Cb.submat(new Rect(aTopLeft, aBottomRight));
            regionB_Cb = Cb.submat(new Rect(bTopLeft, bBottomRight));
            regionC_Cb = Cb.submat(new Rect(cTopLeft, cBottomRight));
        }

        @Override
        public Mat processFrame(Mat input) {
            inputToCb(input);
            averageA = (int) Core.mean(regionA_Cb).val[0];
            averageB = (int) Core.mean(regionB_Cb).val[0];
            averageC = (int) Core.mean(regionC_Cb).val[0];

            //A
            Imgproc.rectangle(input, aTopLeft, aBottomRight, RED,2);
            //B
            Imgproc.rectangle(input, bTopLeft, bBottomRight, BLUE,2);
            //C
            Imgproc.rectangle(input, cTopLeft, cBottomRight, GREEN,2);




            return input;
        }


        public int getAverage() {
            return averageA;
        }

        public int getRegionA() {
            return averageA;
        }

        public int getRegionB() { return averageB; }

        public int getRegionC() { return averageC; }

        public int getReg;

    }
}







/*
Final Notes from Andy: Apressed sets the slide puller to optimum pickup, Bpressed sets slide puller to optimum travel and same concept with elevator motors just with Xpressed and Ypressed. Just need to do cleanup and redo elevator motors since we added another one
*/