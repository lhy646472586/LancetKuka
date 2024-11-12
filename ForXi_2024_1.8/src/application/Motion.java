package application;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import javax.inject.Inject;
import javax.inject.Named;

import com.kuka.common.StatisticTimer;
import com.kuka.common.StatisticTimer.OneTimeStep;
import com.kuka.common.ThreadUtil;
import com.kuka.connectivity.motionModel.smartServoLIN.ISmartServoLINRuntime;
import com.kuka.connectivity.motionModel.smartServoLIN.SmartServoLIN;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.AbstractFrame;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.Frame;
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.XyzAbcTransformation;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;

/**
 * This sample activates a SmartServoLIN motion in position control mode, sends a sequence of Cartesian set points,
 * describing a sine function in z-direction and evaluates the statistic timing.
 */
public class Motion extends RoboticsAPIApplication
{
    private LBR _lbr;
    private Tool _toolAttachedToLBR;
    private LoadData _loadData;
    private ISmartServoLINRuntime _smartServoLINRuntime = null;
	@Named("Tool_2")
	
	@Inject
	private Tool needle;
	
    // Tool Data
    private static final String TOOL_FRAME = "toolFrame";
    private static final double[] TRANSLATION_OF_TOOL = { 0 , 23.0, 119.8,0,1.57,0 };
    private static final double MASS = 0;
    private static final double[] CENTER_OF_MASS_IN_MILLIMETER = { 0, 0, 100 };

    private static final int NUM_RUNS = 600;
    private static final double AMPLITUDE = 70;
    private static final double FREQENCY = 0.6;

    private static final int MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT = 30;
    private CartesianImpedanceControlMode cic;
    @Override
    public void initialize()
    {
        _lbr = getContext().getDeviceFromType(LBR.class);

        // Create a Tool by Hand this is the tool we want to move with some mass
        // properties and a TCP-Z-offset of 100.
        _loadData = new LoadData();
        _loadData.setMass(MASS);
        _loadData.setCenterOfMass(
                CENTER_OF_MASS_IN_MILLIMETER[0], CENTER_OF_MASS_IN_MILLIMETER[1],
                CENTER_OF_MASS_IN_MILLIMETER[2]);
        _toolAttachedToLBR = new Tool("Tool", _loadData);

        XyzAbcTransformation trans = XyzAbcTransformation.ofRad(
                TRANSLATION_OF_TOOL[0], TRANSLATION_OF_TOOL[1],
                TRANSLATION_OF_TOOL[2],TRANSLATION_OF_TOOL[3], TRANSLATION_OF_TOOL[4],TRANSLATION_OF_TOOL[5]);
        ObjectFrame aTransformation = _toolAttachedToLBR.addChildFrame(TOOL_FRAME
                + "(TCP)", trans);
        _toolAttachedToLBR.setDefaultMotionFrame(aTransformation);
        // Attach tool to the robot
        _toolAttachedToLBR.attachTo(_lbr.getFlange());
        needle.attachTo(_lbr.getFlange());
    }

    @Override
    public void run()
    {
        getLogger().info("Move to start position.");
//        _lbr.move(ptp(0., Math.PI / 180 * 30., 0., -Math.PI / 180 * 60., 0.,
//                Math.PI / 180 * 90., 0.).setJointVelocityRel(0.1));

        AbstractFrame initialPosition = _lbr.getCurrentCartesianPosition(needle.getFrame("/tcp_2"));

        // Create a new smart servo linear motion
        SmartServoLIN aSmartServoLINMotion = new SmartServoLIN(initialPosition);

        aSmartServoLINMotion.setMinimumTrajectoryExecutionTime(20e-3);

        getLogger().info("Starting the SmartServoLIN in position control mode");
        _lbr.getFlange().moveAsync(aSmartServoLINMotion);

        getLogger().info("Get the runtime of the SmartServoLIN motion");
        _smartServoLINRuntime = aSmartServoLINMotion.getRuntime();

        StatisticTimer timing = new StatisticTimer();

        // Start the smart servo lin sine movement
        timing = startSineMovement(_smartServoLINRuntime, timing);

        ThreadUtil.milliSleep(1000);

        getLogger().info("Print statistic timing");
        getLogger()
                .info(getClass().getName() + _smartServoLINRuntime.toString());

        getLogger().info("Stop the SmartServoLIN motion");
        _smartServoLINRuntime.stopMotion();

        // Statistic Timing of sine movement loop
        if (timing.getMeanTimeMillis() > 150)
        {
            getLogger()
                    .info("Statistic Timing is unexpected slow, you should try to optimize TCP/IP Transfer");
            getLogger()
                    .info("Under Windows, you should play with the registry, see the e.g. user manual");
        }
    }

    private StatisticTimer startSineMovement(
            ISmartServoLINRuntime smartServoLINRuntime, StatisticTimer timing)
    {
        Frame aFrame = smartServoLINRuntime.getCurrentCartesianDestination(needle.getFrame("/tcp_2"));

        getLogger().info("Do sine movement");
        try
        {
        	 cic = new CartesianImpedanceControlMode();
        	 cic.parametrize(CartDOF.ROT).setStiffness(300);
        	 cic.parametrize(CartDOF.X).setStiffness(5000);
        	 cic.parametrize(CartDOF.Y).setStiffness(5000);
        	 cic.parametrize(CartDOF.Z).setStiffness(5000);
        	
//            double omega = FREQENCY * 2 * Math.PI * 1e-9;
//            long startTimeStamp = System.nanoTime();

            for (int i = 0; i < NUM_RUNS; ++i)
            {
                final OneTimeStep aStep = timing.newTimeStep();
                Frame aFrame_Current = smartServoLINRuntime.getCurrentCartesianDestination(needle.getFrame("/tcp_2"));
                // ///////////////////////////////////////////////////////
                // Insert your code here
                // e.g Visual Servoing or the like
                // Synchronize with the realtime system
               
                
                ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);

                // Update the smart servo LIN runtime
                smartServoLINRuntime.updateWithRealtimeSystem();

//                double curTime = System.nanoTime() - startTimeStamp;
//                double sinArgument = omega * curTime;

                // Compute the sine function
                Frame destFrame = new Frame(aFrame);
                //�?制z�?�?�?�?�
                destFrame.setX(aFrame_Current.getX());
                destFrame.setY(aFrame_Current.getY());
//                destFrame.setZ(AMPLITUDE * Math.sin(sinArgument));
          
                // Set new destination
                smartServoLINRuntime.setDestination(aFrame_Current);
                //final CartesianImpedanceControlMode cartImp = (CartesianImpedanceControlMode) controlMode;
//                smartServoLINRuntime.changeControlModeSettings(cic);
                aStep.end();
            }

        }
        catch (Exception e)
        {
            getLogger().error(e.getLocalizedMessage());
            e.printStackTrace();
        }
        return timing;
    }

    /**
     * Main routine, which starts the application.
     * 
     * @param args
     *            arguments
     */
    public static void main(String[] args)
    {
        Motion app = new Motion();
        app.runApplication();
    }

}
