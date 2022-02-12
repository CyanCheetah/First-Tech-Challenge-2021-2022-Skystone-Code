package org.firstinspires.ftc.teamcode;

//This one's the source for most
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

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
import org.openftc.easyopencv.OpenCvWebcam;



@Autonomous
public class CyanVlogsBlueCarousel extends LinearOpMode {




    private void PIDController(double[] pwr, DcMotor[] mtrs, int[] tarPos) {  //needs brake, runwithoutencoder, and reverse
        double r = getRuntime();
        int mtrIndex = 0;
        boolean robotIsRunning = true;
        for (DcMotor mtr:mtrs) {
            mtr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            mtr.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            mtr.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            mtr.setPower(pwr[mtrIndex]);
            mtrIndex += 1;

        }
        if (mtrs.length == 4) {

            while (Math.abs(mtrs[0].getCurrentPosition())<Math.abs(tarPos[0])||Math.abs(mtrs[1].getCurrentPosition())<Math.abs(tarPos[1])||Math.abs(mtrs[2].getCurrentPosition())<Math.abs(tarPos[2])||Math.abs(mtrs[3].getCurrentPosition())<Math.abs(tarPos[3])) {
                if (!opModeIsActive()) {
                    break;
                }
                telemetry.addLine("Robot is running");
                telemetry.update();
                idle();
            }
        }
        if (mtrs.length == 1) {

            while (Math.abs(mtrs[0].getCurrentPosition())<Math.abs(tarPos[0])) {
                if (!opModeIsActive()) {
                    break;
                }
                telemetry.addLine("Robot is running");
                telemetry.update();
                idle();
            }
        }
        for (DcMotor mtr :mtrs) {
            mtr.setPower(0);
        }
    }
    OpenCvCamera webcam;
    // OpenCvCamera webcam2;
    TripleRectPipeline pipeline;
    // SamplePipeline cyanline;
    private DcMotor bottoml;
    private DcMotor bottomr;
    private DcMotor frontl;
    private DcMotor frontr;
    private  DcMotor ElevatorMotor;
    private CRServo  servo;
    private DcMotor CarouselMotor;
    private int ticksPerRevolution = 145  + 1/10;

    String location = null;

    int a;
    int b;
    int c;
    float pwr = 0.35f;
    public void sleepWhileBusy() {
        while (frontr.isBusy() || frontl.isBusy() || bottoml.isBusy() || bottomr.isBusy()) {


        }
    }



    @Override
    public void runOpMode() {
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "OpenCvCamera"), cameraMonitorViewId);
        // webcam2 = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "OpenCvCamera"), cameraMonitorViewId);

        pipeline = new TripleRectPipeline();
        webcam.setPipeline(pipeline);
        frontl = hardwareMap.get(DcMotor.class, "frontl");
        frontr = hardwareMap.get(DcMotor.class, "frontr");
        bottoml = hardwareMap.get(DcMotor.class, "bottoml");
        bottomr = hardwareMap.get(DcMotor.class, "bottomr");
        ElevatorMotor = hardwareMap.get(DcMotor.class, "ElevatorMotor");
        servo = hardwareMap.get(CRServo.class, "servo");


        CarouselMotor = hardwareMap.get(DcMotor.class, "CarouselMotor");

        DcMotor[] driveMotors = {frontl, frontr, bottoml, bottomr};
        if (!opModeIsActive()) {

            frontl.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            frontr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bottoml.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            bottomr.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
            ElevatorMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


            frontl.setDirection(DcMotorSimple.Direction.REVERSE);
            frontr.setDirection(DcMotorSimple.Direction.FORWARD);
            ElevatorMotor.setDirection(DcMotor.Direction.FORWARD);
            bottoml.setDirection(DcMotorSimple.Direction.REVERSE);
            bottomr.setDirection(DcMotorSimple.Direction.FORWARD);

            frontl.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            frontr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bottoml.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            bottomr.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        }


        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                //1280 x 720 resolution, aka 720p
                webcam.startStreaming(1280, 720, OpenCvCameraRotation.UPRIGHT);

            }

            @Override
            public void onError(int errorCode) {

            }
        });
        waitForStart();

        if (opModeIsActive()) {


            while (pipeline.getRegionA() == 0 && pipeline.getRegionB() == 0 && pipeline.getRegionC() == 0) {

                sleep(1000);
                telemetry.addData("Box A", pipeline.getRegionA());
                telemetry.addData("Box B", pipeline.getRegionB());
                telemetry.addData("Box C", pipeline.getRegionC());
                telemetry.update();

                a = pipeline.getRegionA();
                b = pipeline.getRegionB();
                c = pipeline.getRegionC();
            }
            sleep(500);

            a = pipeline.getRegionA();
            b = pipeline.getRegionB();
            c = pipeline.getRegionC();
            sleep(500); //Safety extra check

            if (a < b && a < c) {
                location = "a"; //least cB

            }
            if (b < a && b < c) {
                location = "b"; //least cB
            }
            if (c < b && c < a) {
                location = "c"; //least cB
            }

            //Get to hub

            PIDController(new double[]{-pwr, pwr, pwr, -pwr}, new DcMotor[]{frontl, frontr, bottoml, bottomr}, new int[]{-30, 30, 30, -30});
            //Turn in place 90 Degrees
            //Backwards
            PIDController(new double[]{-pwr, -pwr, -pwr, -pwr}, new DcMotor[]{frontl, frontr, bottoml, bottomr}, new int[]{-400, -400, -400, -400});
            //strafe
            //PIDController(new double[]{-pwr, pwr, pwr, -pwr}, new DcMotor[]{frontl, frontr, bottoml, bottomr}, new int[]{-30, 30, 30, -30});
            //Turn in place 90 Degrees
            PIDController(new double[]{-pwr, pwr, -pwr, pwr}, new DcMotor[]{frontl, frontr, bottoml, bottomr}, new int[]{-325, 325, -325, 325});
            //Square Up
            frontl.setPower(-0.3);
            frontr.setPower(-0.3);
            bottoml.setPower(-0.3);
            bottomr.setPower(-0.3);
            for (DcMotor mtr : driveMotors) {
                mtr.setTargetPosition(-5000);
            }
            sleep(2000);
            for (DcMotor mtr :
                    driveMotors) {
                mtr.setPower(0);
            }
            //Head to hub

            int headToHubDistance = 300; //Prev 390

            if (location == "a") {
                PIDController(new double[]{pwr, pwr, pwr, pwr}, new DcMotor[]{frontl, frontr, bottoml, bottomr}, new int[]{headToHubDistance-110, headToHubDistance-110, headToHubDistance-110, headToHubDistance-110});
            }
            if (location == "b") {
                PIDController(new double[]{pwr, pwr, pwr, pwr}, new DcMotor[]{frontl, frontr, bottoml, bottomr}, new int[]{headToHubDistance - 140, headToHubDistance - 140, headToHubDistance - 140, headToHubDistance - 140});
            }
            if (location == "c") {
                PIDController(new double[]{pwr, pwr, pwr, pwr}, new DcMotor[]{frontl, frontr, bottoml, bottomr}, new int[]{headToHubDistance - 180, headToHubDistance - 180, headToHubDistance - 180, headToHubDistance - 180});
            }


            sleep(1000);
            //PIDController(new double[] {pwr,pwr,pwr,pwr}, new DcMotor[] {frontl,frontr,bottoml,bottomr}, new int[] {60, 60, 60, 60});
            if (a < b && a < c) {
                location = "a";
                telemetry.addLine("The ARTIFACT is in BOX A");

                PIDController(new double[]{1}, new DcMotor[]{ElevatorMotor}, new int[]{3300});
                // sleep(500);
                //  servo.setPower(-0.5);
                //   sleep(350);
                //   servo.setPower(0);

            }

            if (b < a && b < c) {
                location = "b";
                telemetry.addLine("The ARTIFACT is in BOX B");

                PIDController(new double[]{1}, new DcMotor[]{ElevatorMotor}, new int[]{6766});
                //      sleep(500);
                //       servo.setPower(-0.5);
                //       sleep(350);
                //     servo.setPower(0);

            }
            if (c < a && c < b) {
                location = "c";
                telemetry.addLine("The ARTIFACT is in BOX C");


                PIDController(new double[]{1}, new DcMotor[]{ElevatorMotor}, new int[]{11600});
            }

            //Arm the box1
            servo.setPower(-0.5);
            sleep(300);
            servo.setPower(0);
            //Head to hub after armed
            int headToHubDistance2 = 200;
            if (location == "c") {
                //Go forward
                PIDController(new double[]{.3, .3, .3, .3}, new DcMotor[]{frontl, frontr, bottoml, bottomr}, new int[]{headToHubDistance2, headToHubDistance2, headToHubDistance2, headToHubDistance2});

            }if (location == "b") {
                //Go forward
                PIDController(new double[]{.3, .3, .3, .3}, new DcMotor[]{frontl, frontr, bottoml, bottomr}, new int[]{headToHubDistance2+30, headToHubDistance2+30, headToHubDistance2+30, headToHubDistance2+30});

            }
            if (location == "a") {
                PIDController(new double[]{.3, .3, .3, .3}, new DcMotor[]{frontl, frontr, bottoml, bottomr}, new int[]{headToHubDistance2+60,headToHubDistance2+60 , headToHubDistance2+60,headToHubDistance2+60 });

                //deploy the box
            }

            PIDController(new double[]{-pwr, -pwr, -pwr, -pwr}, new DcMotor[]{frontl, frontr, bottoml, bottomr}, new int[]{-200, -200, -200, -200});

            servo.setPower(0.4);
            sleep(500);
            servo.setPower(0);


            //Lower Elevator based on what it was initially raised to
            if (location == "c") {
                PIDController(new double[]{-0.75}, new DcMotor[]{ElevatorMotor}, new int[]{-10800});
            } else if (location == "b") {
                PIDController(new double[]{-0.75}, new DcMotor[]{ElevatorMotor}, new int[]{-5900});

            } else {
                PIDController(new double[]{-1}, new DcMotor[]{ElevatorMotor}, new int[]{-3100});

            }
            //Just go forward
            PIDController(new double[]{pwr, pwr, pwr, pwr}, new DcMotor[]{frontl, frontr, bottoml, bottomr}, new int[]{34, 34, 34, 34});

            //turn in place right
            PIDController(new double[]{pwr, -pwr, pwr, -pwr}, new DcMotor[]{frontl, frontr, bottoml, bottomr}, new int[]{350, -350, 350, -350});

            //go forward, full speed demon babeyyyyyY!
            PIDController(new double[]{0.5, 0.5, 0.5, 0.5}, new DcMotor[]{frontl, frontr, bottoml, bottomr}, new int[]{600, 600, 600, 600});

            //Slow down to square up against the wall
            PIDController(new double[]{.25, .25, .25, .25}, new DcMotor[]{frontl, frontr, bottoml, bottomr}, new int[]{500, 500, 500, 500});

            //square up
            frontl.setPower(0.3);
            frontr.setPower(0.3);
            bottoml.setPower(0.3);
            bottomr.setPower(0.3);
            for (DcMotor mtr : driveMotors) {
                mtr.setTargetPosition(-5000);
            }
            sleep(800);
            for (DcMotor mtr :
                    driveMotors) {
                mtr.setPower(0);
            }
            //go back
            PIDController(new double[]{-pwr, -pwr, -pwr, -pwr}, new DcMotor[]{frontl, frontr, bottoml, bottomr}, new int[]{-50, -50, -50, -50});
            //turn in place
            PIDController(new double[]{pwr, -pwr, pwr, -pwr}, new DcMotor[]{frontl, frontr, bottoml, bottomr}, new int[]{290, -290, 290, -290});


            //strafe
            PIDController(new double[]{-.25, .25, .25, -.25}, new DcMotor[]{frontl, frontr, bottoml, bottomr}, new int[]{-25, 25, 25, -25});



            //go forward

            frontl.setPower(0.1);
            frontr.setPower(0.1);
            bottoml.setPower(0.1);
            bottomr.setPower(0.1);
            for (DcMotor mtr : driveMotors) {
                mtr.setTargetPosition(-5000);
            }
            if (location == "c"); {
                sleep(900);
            }
            sleep(1800);
            for (DcMotor mtr :
                    driveMotors) {
                mtr.setPower(0);
            }

//Spam Carousel
            // PIDController(new double[]{1}, new DcMotor[]{CarouselMotor}, new int[]{})
            //Pseudo Time Based Carousel Movement
            CarouselMotor.setTargetPosition(10000);
            CarouselMotor.setPower(1);
            sleep(3200);
            CarouselMotor.setPower(0);
            //go back to park
            PIDController(new double[]{-pwr, -pwr, -pwr, -pwr}, new DcMotor[]{frontl, frontr, bottoml, bottomr}, new int[]{-350, -350, -350, -350});

            //strafe right
            PIDController(new double[]{-pwr, pwr, pwr, -pwr}, new DcMotor[]{frontl, frontr, bottoml, bottomr}, new int[]{-70, 70, 70, -70});

        }//if OpMode Is Active

        stop();


    }

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
        Point aTopLeft = new Point(70, 140);
        Point aBottomRight = new Point(220, 290);

        Point bTopLeft = new Point(570, 140);
        Point bBottomRight = new Point(720, 290);

        Point cTopLeft = new Point(1030, 140);
        Point cBottomRight = new Point(1180, 295);

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
