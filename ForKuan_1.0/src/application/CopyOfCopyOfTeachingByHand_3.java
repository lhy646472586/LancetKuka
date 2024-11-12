package application;

import static com.kuka.roboticsAPI.motionModel.BasicMotions.ptp;

import javax.inject.Inject;

import com.kuka.common.StatisticTimer;
import com.kuka.common.StatisticTimer.OneTimeStep;
import com.kuka.common.ThreadUtil;
import com.kuka.connectivity.motionModel.smartServo.ISmartServoRuntime;
import com.kuka.connectivity.motionModel.smartServo.ServoMotion;
import com.kuka.connectivity.motionModel.smartServo.SmartServo;
import com.kuka.roboticsAPI.applicationModel.RoboticsAPIApplication;
import com.kuka.roboticsAPI.deviceModel.JointEnum;
import com.kuka.roboticsAPI.deviceModel.JointPosition;
import com.kuka.roboticsAPI.deviceModel.LBR;
import com.kuka.roboticsAPI.geometricModel.CartDOF;
import com.kuka.roboticsAPI.geometricModel.LoadData;
import com.kuka.roboticsAPI.geometricModel.ObjectFrame;
import com.kuka.roboticsAPI.geometricModel.Tool;
import com.kuka.roboticsAPI.geometricModel.math.XyzAbcTransformation;
import com.kuka.roboticsAPI.motionModel.IMotionContainer;
import com.kuka.roboticsAPI.motionModel.controlModeModel.CartesianImpedanceControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.IMotionControlMode;
import com.kuka.roboticsAPI.motionModel.controlModeModel.JointImpedanceControlMode;
//import com.kuka.statuscontroller.api.IStatusController;

/**
 * This example activates a SmartServo motion in Cartesian impedance control mode, sends a sequence of Cartesian set
 * points and modifies compliance parameters during the motion.
 * 
 */


public class CopyOfCopyOfTeachingByHand_3 extends RoboticsAPIApplication
{
    private LBR _lbr;
    private Tool _toolAttachedToLBR;
    private LoadData _loadData;
//    @Inject
//    private IStatusController statusMonitor;

    // Tool Data
    private static final String TOOL_FRAME = "toolFrame";
    private static final double[] TRANSLATION_OF_TOOL = { 0, 0, 100 };
    private static final double MASS = 0;
    private static final double[] CENTER_OF_MASS_IN_MILLIMETER = { 0, 0, 100 };

    private static final int NUM_RUNS = 600;
    private static final double AMPLITUDE = 0.2;
    private static final double FREQENCY = 0.1;
	public static boolean bModeAix=false;
    private static final int MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT = 30;

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

        XyzAbcTransformation trans = XyzAbcTransformation.ofTranslation(
                TRANSLATION_OF_TOOL[0], TRANSLATION_OF_TOOL[1],
                TRANSLATION_OF_TOOL[2]);
        ObjectFrame aTransformation = _toolAttachedToLBR.addChildFrame(TOOL_FRAME
                + "(TCP)", trans);
        _toolAttachedToLBR.setDefaultMotionFrame(aTransformation);
        // Attach tool to the robot
        _toolAttachedToLBR.attachTo(_lbr.getFlange());
    }

    /**
     * Move to an initial Position WARNING: MAKE SURE, THAT the pose is collision free.
     */
    public void moveToInitialPosition()
    {
        _lbr.move(ptp(0., Math.PI / 180 * 30., 0., -Math.PI / 180 * 60., 0.,
                Math.PI / 180 * 90., 0.).setJointVelocityRel(0.1));
        /* Note: The Validation itself justifies, that in this very time instance, the load parameter setting was
         * sufficient. This does not mean by far, that the parameter setting is valid in the sequel or lifetime of this
         * program */
        if (!ServoMotion.validateForImpedanceMode(_lbr))
        {
            getLogger()
                    .info("Validation of torque model failed - correct your mass property settings");
            getLogger()
                    .info("Servo motion will be available for position controlled mode only, until validation is performed");
        }
    }

    /**
     * Creates a smartServo motion with the given control mode and moves around.
     * 
     * @param controlMode
     *            the control mode which shall be used
     * @see {@link CartesianImpedanceControlMode}
     */

    protected void runSmartServoMotion(final IMotionControlMode controlMode)
    {
        final boolean doDebugPrints = false;

        final JointPosition initialPosition = new JointPosition(
                _lbr.getCurrentJointPosition());

        final SmartServo aSmartServoMotion = new SmartServo(initialPosition);

        // Set the motion properties to 10% of the systems abilities
        aSmartServoMotion.setJointAccelerationRel(0.1);
        aSmartServoMotion.setJointVelocityRel(0.1);

        aSmartServoMotion.setMinimumTrajectoryExecutionTime(20e-3);

        getLogger().info("Starting the SmartServo in " + controlMode);
        IMotionContainer container = _toolAttachedToLBR.getDefaultMotionFrame().moveAsync(
                aSmartServoMotion.setMode(controlMode));

        // Fetch the Runtime of the Motion part
        final ISmartServoRuntime theSmartServoRuntime = aSmartServoMotion
                .getRuntime();

        // create an JointPosition Instance, to play with
        final JointPosition destination = new JointPosition(
                _lbr.getJointCount());

        // For Roundtrip time measurement...
        final StatisticTimer timing = new StatisticTimer();
        try
        {
            // do a cyclic loop
            // Refer to some timing...
            // in nanosec
            final double omega = FREQENCY * 2 * Math.PI * 1e-9;
            final long startTimeStamp = System.nanoTime();

            while(!container.isFinished())
            {
                // Timing - draw one step
                final OneTimeStep aStep = timing.newTimeStep();
                // ///////////////////////////////////////////////////////
                // Insert your code here
                // e.g Visual Servoing or the like
                // emulate some computational effort - or waiting for external
                // stuff
                ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);

                theSmartServoRuntime.updateWithRealtimeSystem();

                // Get the measured position in cartesian...
                final JointPosition curMsrJntPose = theSmartServoRuntime
                        .getAxisQMsrOnController();
                final JointPosition currentPos = _lbr.getCurrentJointPosition();
                
   
                if(Math.abs(Math.toDegrees(currentPos.get(JointEnum.J4))) > 60){
                	 if (controlMode instanceof JointImpedanceControlMode)
                       {
                           final JointImpedanceControlMode jointImp = (JointImpedanceControlMode) controlMode;
                           double para = 0;
                           if(currentPos.get(JointEnum.J4) > 0){
                        	   para = 30.0 / (Math.abs(Math.toRadians(120) - currentPos.get(JointEnum.J4)));
                           }
                           else{
                        	   para = 30.0 / (Math.abs(Math.toRadians(-120) - currentPos.get(JointEnum.J4)));
                           }
//                           800.0 / Math.abs(Math.toRadians(120) - currentPos.get(JointEnum.J4));
                           
                           jointImp.setStiffness(1, 1, 1, para, 1, 1, 1);
                           System.out.println(para);
                           jointImp.setDamping(0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5);
                           // Send the new Stiffness settings down to the
                           // controller
                           theSmartServoRuntime
                                   .changeControlModeSettings(jointImp);
                       	bModeAix=false;
                       }
                }
                else if(Math.abs(Math.toDegrees(currentPos.get(JointEnum.J5))) > 140){
               	 	if (controlMode instanceof JointImpedanceControlMode)
               	 	{
               	 		final JointImpedanceControlMode jointImp = (JointImpedanceControlMode) controlMode;
               	 		jointImp.setStiffness(1, 1, 1, 1, 700.0 / Math.abs(170 - Math.abs(Math.toDegrees(currentPos.get(JointEnum.J5)))), 1, 1);
                        System.out.println(500.0 / Math.abs(170 - Math.abs(Math.toDegrees(currentPos.get(JointEnum.J5)))));
               	 		jointImp.setDamping(0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5);
               	 		// Send the new Stiffness settings down to the
               	 		// controller
               	 		theSmartServoRuntime
               	 		.changeControlModeSettings(jointImp);
               	 	bModeAix=false;
               	 	} 	
                }
                else{
                	
                	
                	
                	//if(bModeAix==false){
//           	 		final JointImpedanceControlMode jointImp = (JointImpedanceControlMode) controlMode;
//           	 		jointImp.setStiffness(1, 1, 1, 1, 1, 1, 1);
//           	 		jointImp.setDamping(0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5);
//           	 		// Send the new Stiffness settings down to the
//           	 		// controller
//           	 		theSmartServoRuntime
//           	 		.changeControlModeSettings(jointImp);
//                	//}
//                	
//                	bModeAix=true;
//                	
////                	final CartesianImpedanceControlMode cartImp = (CartesianImpedanceControlMode) controlMode;
////                    cartImp.parametrize(CartDOF.TRANSL).setStiffness(2);
////                    cartImp.parametrize(CartDOF.ROT).setStiffness(30);
////           	 		theSmartServoRuntime
////           	 		.changeControlModeSettings(cartImp);
//                    getLogger()
//                    .info("CartesianImpedanceControlMode......");
                	if (controlMode instanceof JointImpedanceControlMode)
               	 	{
               	 		final JointImpedanceControlMode jointImp = (JointImpedanceControlMode) controlMode;
               	 		jointImp.setStiffness(1, 1, 1, 1, 1, 1, 1);
               	 		jointImp.setDamping(0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5);
               	 		// Send the new Stiffness settings down to the
               	 		// controller
               	 		theSmartServoRuntime
               	 		.changeControlModeSettings(jointImp);
               	 	} 	
                }
                
                destination.set(currentPos);
//                currentPos.get(joint)
                theSmartServoRuntime
                        .setDestination(destination);

                aStep.end();


            }
        }
        catch (final Exception e)
        {
            getLogger().info(e.getLocalizedMessage());
            e.printStackTrace();
        }

        //Print statistics and parameters of the motion
        getLogger().info("Displaying final states after loop "
                + controlMode.getClass().getName());
        getLogger().info(getClass().getName() + theSmartServoRuntime.toString());
        // Stop the motion
        theSmartServoRuntime.stopMotion();
        getLogger().info("Statistic Timing of Overall Loop " + timing);
        if (timing.getMeanTimeMillis() > 150)
        {
            getLogger().info("Statistic Timing is unexpected slow, you should try to optimize TCP/IP Transfer");
            getLogger().info("Under Windows, you should play with the registry, see the e.g. the RealtimePTP Class javaDoc for details");
        }

    }

    /**
     * Create the CartesianImpedanceControlMode class for motion parameterisation.
     * 
     * @see {@link CartesianImpedanceControlMode}
     * @return the created control mode
     */
    protected CartesianImpedanceControlMode createCartImp()
    {
        final CartesianImpedanceControlMode cartImp = new CartesianImpedanceControlMode();
        //cartImp.parametrize(CartDOF.TRANSL).setStiffness(1000.0);
        cartImp.parametrize(CartDOF.X).setStiffness(2);
        cartImp.parametrize(CartDOF.Y).setStiffness(2);
        cartImp.parametrize(CartDOF.Z).setStiffness(2);
        cartImp.parametrize(CartDOF.ROT).setStiffness(30.0);
        cartImp.setNullSpaceStiffness(2);
        // For your own safety, shrink the motion abilities to useful limits
        cartImp.setMaxPathDeviation(50., 50., 50., 50., 50., 50.);
        return cartImp;
    }
    
    
    /**
     * Create the CartesianImpedanceControlMode class for motion parameterisation.
     * 
     * @see {@link CartesianImpedanceControlMode}
     * @return the created control mode
     */
    protected JointImpedanceControlMode createJointImp()
    {
        final JointImpedanceControlMode jointImp = new JointImpedanceControlMode(1, 1, 1, 1, 1, 1, 1);
//        jointImp.setStiffness(1, 1, 1, 1, 1, 1, 1);
        jointImp.setDamping(0.5, 0.5, 0.5, 0.5, 0.5, 0.5, 0.5);
        return jointImp;
    }

    /** Sample to switch the motion control mode. */
    protected void switchMotionControlMode()
    {
        getLogger().info("Switch Motion Control Mode Sample");
        final boolean debugPrintoutFlag = false;
        /* Prepare two control modes for the sample */
        final JointImpedanceControlMode cartImp = createJointImp();

        final JointPosition initialPosition = new JointPosition(
                _lbr.getCurrentJointPosition());

        final SmartServo firstSmartServoMotion = new SmartServo(initialPosition);

        // Set the motion properties to 10% of the systems abilities
        firstSmartServoMotion.setJointAccelerationRel(0.1);
        firstSmartServoMotion.setJointVelocityRel(0.1);
        firstSmartServoMotion.setMode(cartImp);

        getLogger().info("Starting the SmartServo in " + cartImp);
        _toolAttachedToLBR.getDefaultMotionFrame().moveAsync(firstSmartServoMotion);

        // Fetch the Runtime of the Motion part
        // NOTE: the Runtime will exist AFTER motion command was issued
        final ISmartServoRuntime theFirstRuntime = firstSmartServoMotion
                .getRuntime();

        /* Do Interaction with that mode Run set points etc... */
        initialPosition.set(2, -1);
        theFirstRuntime.setDestination(initialPosition);

        /* Here: Just wait, until fine interpolation has finished */
        while (!theFirstRuntime.isDestinationReached())
        {
            ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);
            theFirstRuntime.updateWithRealtimeSystem();
            if (debugPrintoutFlag)
            {
                getLogger().info("Waiting for reaching goal "
                        + theFirstRuntime);
            }
        }

        // Open second Motion
        for (int i = 0; i < initialPosition.getAxisCount(); i++)
        {
            initialPosition.set(i, initialPosition.get(i) + 0.1);
        }

        final SmartServo secondSmartServoMotion = new SmartServo(initialPosition);

        secondSmartServoMotion.setJointAccelerationRel(0.2);
        secondSmartServoMotion.setJointVelocityRel(0.1);

        // / Activate the Motion -- it will become truely active, as the first
        // one vanishes
        // Set the control mode as member of the realtime motion
        _toolAttachedToLBR.getDefaultMotionFrame().moveAsync(secondSmartServoMotion);

        getLogger().info("Now blending over to -> Sending Stop Request "
                );

        // ///////////////////////////////////
        // Now blend over - stop the first,
        // the second will immediately take over
        theFirstRuntime.stopMotion();

        final ISmartServoRuntime theSecondRuntime = secondSmartServoMotion
                .getRuntime();
        theSecondRuntime.setDestination(initialPosition);
        /* do further computations... */

        /* Here: Just wait, until fine interpolation has finished */
        while (!theSecondRuntime.isDestinationReached())
        {
            ThreadUtil.milliSleep(MILLI_SLEEP_TO_EMULATE_COMPUTATIONAL_EFFORT);
            theSecondRuntime.updateWithRealtimeSystem();
            if (debugPrintoutFlag)
            {
                getLogger().info("Waiting for reaching goal Runtime 2 "
                        + theSecondRuntime);
            }
        }

        theSecondRuntime.stopMotion();

        getLogger().info("Result of Motion 1 " + theFirstRuntime);
        getLogger().info("Result of Motion 2 " + theSecondRuntime);
    }

    @Override
    public void run()
    {
//    	_lbr.getExternalForceTorque(_lbr.getFlange());
    	
    	
//        moveToInitialPosition();

        // Initialize Cartesian impedance mode       
        final JointImpedanceControlMode jointImp = createJointImp();
        
        // Initialize Cartesian impedance mode       
//        final CartesianImpedanceControlMode cartImp = createCartImp();

        runSmartServoMotion(jointImp);

        // Return to initial position
        //moveToInitialPosition();

       // final PositionControlMode positionCtrlMode = new PositionControlMode();

        //runSmartServoMotion(positionCtrlMode);

        //moveToInitialPosition();

        //switchMotionControlMode();
        
//        System.out.println(statusMonitor.getActiveStatusGroups());
    }

    /**
     * Main routine, which starts the application.
     * 
     * @param args
     *            arguments
     */
    public static void main(final String[] args)
    {
        final CopyOfCopyOfTeachingByHand_3 app = new CopyOfCopyOfTeachingByHand_3();
        app.runApplication();
    }
}
