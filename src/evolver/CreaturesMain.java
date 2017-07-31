package evolver;

import com.jme3.app.SimpleApplication;
import com.jme3.asset.TextureKey;

import com.jme3.bullet.BulletAppState;
import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.CapsuleCollisionShape;
import com.jme3.bullet.control.RigidBodyControl;
import com.jme3.bullet.joints.ConeJoint;
import com.jme3.bullet.joints.HingeJoint;
import com.jme3.bullet.joints.PhysicsJoint;
import com.jme3.bullet.joints.SixDofJoint;
import com.jme3.bullet.joints.motors.RotationalLimitMotor;
import com.jme3.bullet.joints.motors.TranslationalLimitMotor;

import com.jme3.input.KeyInput;
import com.jme3.input.controls.ActionListener;
import com.jme3.input.controls.KeyTrigger;
import com.jme3.input.controls.MouseButtonTrigger;

import com.jme3.light.AmbientLight;
import com.jme3.light.DirectionalLight;
import com.jme3.material.Material;

import com.jme3.math.ColorRGBA;
import com.jme3.math.FastMath;
import com.jme3.math.Matrix3f;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;

import com.jme3.renderer.queue.RenderQueue.ShadowMode;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;
import com.jme3.scene.shape.Box;
import com.jme3.scene.shape.Cylinder;
import com.jme3.scene.shape.Sphere;

import com.jme3.shadow.BasicShadowRenderer;
import com.jme3.shadow.DirectionalLightShadowRenderer;
import com.jme3.system.JmeContext;
import com.jme3.texture.Texture;
import com.jme3.util.TangentBinormalGenerator;

import network.Network;
import network.NetworkBuilder;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.HashMap;
import java.util.Map;
import java.util.Random;
import java.util.Set;
import java.util.TreeMap;

import java.util.logging.Level;
import java.util.logging.Logger;


public class CreaturesMain extends SimpleApplication implements ActionListener
{
	/// \brief Main function. Just starts the app.
	public static void main(String[] args)
	{
		CreaturesMain app = new CreaturesMain();

		app.setPauseOnLostFocus(false);
		app.start();

		//This alternate run line runs the code without graphics
		//app.start(JmeContext.Type.Headless);
	}

	@Override
	/// \brief Automatically called when the application starts
	public void simpleInitApp ()
	{
		// Init Logger
		Logger.getLogger ("").setLevel (Level.SEVERE);

		// Init basic environment
		setupLight ();
		setupKeys (m_dvorakMode);
		flyCam.setMoveSpeed (10);

		// Init physics
		m_bulletAppState = new BulletAppState ();

		m_bulletAppState.setSpeed (SIM_SPEED);

		stateManager.attach (m_bulletAppState);
		PhysicsTestHelper.createPhysicsTestWorld (
			rootNode, assetManager, m_bulletAppState.getPhysicsSpace ());
		createRagDoll ();

		//Create initial population of neural network controllers
		setupNetworks (m_evolveMode);
		loadNetwork(m_runningNetworkId);

		singleLineNetworkInput = new double[7];
		singleLineNetworkInput[6] = 1.0f;

		if (m_debugMode)
			m_bulletAppState.setDebugEnabled (true);

		System.out.println("generation: " + m_genCounter);
		System.out.println("evaluating network: " + m_runningNetworkId);
	}


	@Override
	public void simpleUpdate(float tpf_)
	{
		if (m_genCounter > NUM_GENS)
		{
			return;
		}

		if (m_evalCounter > NUM_EVAL_STEPS)
		{
			//calculate fitness
			int tempFitness = 0;
			Vector3f endLoc = m_shoulders.getLocalTranslation ();
			float distanceTraveled = startLoc.distance (endLoc);

			distanceTraveled = Math.round(distanceTraveled * 100);
			tempFitness = (int) distanceTraveled;
			fitnessVals.put (m_runningNetworkId, tempFitness);
			m_evalCounter = 0;
			m_runningNetworkId++;

			nextNetwork ();
		}
		else
		{
			int i = 0;
			while (i < SIM_SPEED)
			{
				m_nn.activate(singleLineNetworkInput);
				i++;
			}

			singleLineNetworkInput[0] = desiredYawVelocity1;
			singleLineNetworkInput[1] = desiredRollVelocity1;
			singleLineNetworkInput[2] = desiredPitchVelocity1;
			singleLineNetworkInput[3] = desiredYawVelocity2;
			singleLineNetworkInput[4] = desiredRollVelocity2;
			singleLineNetworkInput[5] = desiredPitchVelocity2;
			singleLineNetworkInput[6] = 1.0f;

			//Set desired velocities as difference of two output neurons, using a push/pull representation
			desiredYawVelocity1 = (float) m_nn.getNeuronByID(7).getMembranePotential() - (float) m_nn.getNeuronByID(8).getMembranePotential();
			desiredRollVelocity1 = (float) m_nn.getNeuronByID(9).getMembranePotential() - (float) m_nn.getNeuronByID(10).getMembranePotential();
			desiredPitchVelocity1 = (float) m_nn.getNeuronByID(11).getMembranePotential() - (float) m_nn.getNeuronByID(12).getMembranePotential();
			desiredYawVelocity2 = (float) m_nn.getNeuronByID(13).getMembranePotential() - (float) m_nn.getNeuronByID(14).getMembranePotential();
			desiredRollVelocity2 = (float) m_nn.getNeuronByID(15).getMembranePotential() - (float) m_nn.getNeuronByID(16).getMembranePotential();
			desiredPitchVelocity2 = (float) m_nn.getNeuronByID(17).getMembranePotential() - (float) m_nn.getNeuronByID(18).getMembranePotential();

			yawMotor1.setTargetVelocity(desiredYawVelocity1);
			rollMotor1.setTargetVelocity(desiredRollVelocity1);
			pitchMotor1.setTargetVelocity(desiredPitchVelocity1);

			yawMotor2.setTargetVelocity(desiredYawVelocity2);
			rollMotor2.setTargetVelocity(desiredRollVelocity2);
			pitchMotor2.setTargetVelocity(desiredPitchVelocity2);

			m_evalCounter++;
		}
	}

	private void nextNetwork ()
	{
		// Finished the whole population so breed and reset
		if (m_runningNetworkId > POP_SIZE)
		{
			System.out.println ("unsorted map: "+fitnessVals);

			FitnessComparator fc =
				new FitnessComparator (fitnessVals);

			sortedFitnessVals = new TreeMap<Integer,Integer>(fc);
			sortedFitnessVals.putAll(fitnessVals);

			System.out.println("sorted map: "+sortedFitnessVals);
			System.out.println(sortedFitnessVals.firstKey());

			Set sortedKeys = sortedFitnessVals.keySet();

			System.out.println(sortedKeys);

			int numSurviving =
				(int) (POP_SIZE * survivalPercentage);
			int numDying = POP_SIZE - numSurviving;
			Object[] sortedArray = sortedKeys.toArray();

			mostFit = Arrays.copyOfRange (
				sortedArray, 0, numSurviving, Integer[].class);
			leastFit = Arrays.copyOfRange (
				sortedArray, numSurviving, sortedArray.length,
				Integer[].class);

/*
			for (int i = 0; i < mostFit.length; i++) {
				System.out.println("mostFit: " + mostFit[i]);
			}
			for (int i = 0; i < leastFit.length; i++) {
				System.out.println("leastFit: " + leastFit[i]);
			}
*/

			Repopulator rp = new Repopulator ();

			rp.repopulate (mostFit, leastFit);
			fitnessVals.clear ();

			m_runningNetworkId = 1;
			m_genCounter++;

			if (m_genCounter > NUM_GENS)
			{
				System.out.println ("Finished!");
			}
			else
			{
				System.out.println("generation: " + m_genCounter);
				System.out.println("evaluating network: " +
						   m_runningNetworkId);
			}
		}

		// Otherwise load the next network
		loadNetwork (m_runningNetworkId);

		System.out.println("evaluating network: " +
				   m_runningNetworkId);

		m_bulletAppState.getPhysicsSpace().removeAll(
			m_agent.getChild(0));
		m_bulletAppState.getPhysicsSpace().removeAll(
			m_agent.getChild(1));
		m_bulletAppState.getPhysicsSpace().removeAll(
			m_agent.getChild(2));
		m_agent.removeFromParent();

		System.gc();

		//create the next m_agent
		m_agent = new Node();
		createRagDoll();
	}

	/// \brief Set up the lighting
	private void setupLight ()
	{
		viewPort.setBackgroundColor(new ColorRGBA(0.7f, 0.8f, 1f, 1f));

		// We add light so we see the scene
		AmbientLight al = new AmbientLight();
		al.setColor(ColorRGBA.Gray);
		rootNode.addLight(al);

		DirectionalLight sun = new DirectionalLight();
		sun.setDirection(new Vector3f(1, -2.5f, -2).normalizeLocal());
		sun.setColor(ColorRGBA.Gray.mult(1.7f));
		rootNode.addLight(sun);
	}

	/// \brief Set up the input key mappings
	private void setupKeys (boolean dvorakMode_)
	{
		if (dvorakMode_)
		{
			setupDvorakKeys();
			return;
		}

		setupQwertyKeys();
	}

	/// \brief Set up the input key mappings for a qwerty keyboard
	private void setupQwertyKeys ()
	{
		inputManager.addMapping("PitchPlus", new KeyTrigger(KeyInput.KEY_I));
		inputManager.addMapping("PitchMinus", new KeyTrigger(KeyInput.KEY_K));
		inputManager.addMapping("YawPlus", new KeyTrigger(KeyInput.KEY_O));
		inputManager.addMapping("YawMinus", new KeyTrigger(KeyInput.KEY_U));
		inputManager.addMapping("RollPlus", new KeyTrigger(KeyInput.KEY_L));
		inputManager.addMapping("RollMinus", new KeyTrigger(KeyInput.KEY_J));
		inputManager.addMapping("Reset", new KeyTrigger(KeyInput.KEY_R));
		inputManager.addMapping("toggleEnableMotors", new KeyTrigger(KeyInput.KEY_SPACE));

		inputManager.addListener(this, "RollMinus", "RollPlus", "PitchMinus", "PitchPlus", "YawMinus", "YawPlus", "Reset", "toggleEnableMotors");
	}

	/// \brief Set up the input key mappings for a dvorak keyboard
	private void setupDvorakKeys ()
	{
		inputManager.addMapping("PitchPlus", new KeyTrigger(KeyInput.KEY_C));
		inputManager.addMapping("PitchMinus", new KeyTrigger(KeyInput.KEY_T));
		inputManager.addMapping("YawPlus", new KeyTrigger(KeyInput.KEY_R));
		inputManager.addMapping("YawMinus", new KeyTrigger(KeyInput.KEY_G));
		inputManager.addMapping("RollPlus", new KeyTrigger(KeyInput.KEY_N));
		inputManager.addMapping("RollMinus", new KeyTrigger(KeyInput.KEY_H));
		inputManager.addMapping("Reset", new KeyTrigger(KeyInput.KEY_P));
		inputManager.addMapping("toggleEnableMotors", new KeyTrigger(KeyInput.KEY_SPACE));

		inputManager.addListener(this, "RollMinus", "RollPlus", "PitchMinus", "PitchPlus", "YawMinus", "YawPlus", "Reset", "toggleEnableMotors");
	}

	/// \brief Initialize a population of networks
	private void setupNetworks (boolean evolveMode_)
	{
		if (! evolveMode_)
		{
			m_runningNetworkId = NET_ID_TO_EVAL;
			return;
		}

		for (int i = 0; i < POP_SIZE; i++)
		{
			try
			{
				String outDir = NET_DIR;
				String outName = (i + 1) + ".xml";

				File saveFile = new File(outDir + outName);
				BufferedWriter bwOut = new BufferedWriter(
					new FileWriter(saveFile));

				NetworkCreator nc = new NetworkCreator ();
				String netXMLString =
					nc.createNetwork (i + 1, 7, 12);

				bwOut.write(netXMLString);
				bwOut.close();
			}
			catch (IOException exception_)
			{
				System.out.println (exception_.getMessage ());
				System.exit (0);
			}
		}
	}

	/// \brief Load a network from its ID
	private void loadNetwork (int networkId_)
	{
		File file = new File(NET_DIR + networkId_ + ".xml");

		NetworkBuilder networkBuilder = new NetworkBuilder();
		m_nn = networkBuilder.buildNetworkFromFile(file);
		m_nn.setWinnerTakeAll(true);
	}

	/// \brief Create a ragdoll entity.
	/// \todo Move this into its own class
	private void createRagDoll ()
	{
		m_shoulders = createLimb (
			0.2f, 1.0f, new Vector3f (0.00f, 1.5f, 0), true, false);
		Node uArmL = createLimb (
			0.2f, 0.5f, new Vector3f (-0.75f, 0.8f, 0), false, false);
		Node uArmR = createLimb (
			0.2f, 0.5f, new Vector3f (0.75f, 0.8f, 0), false, false);

		lShoulderJoint = join (uArmL, m_shoulders,
				       new Vector3f(-0.75f, 1.4f, 0));
		rShoulderJoint = join (uArmR, m_shoulders,
				       new Vector3f(0.75f, 1.4f, 0));

		pitchMotor1 = lShoulderJoint.getRotationalLimitMotor(0);
		yawMotor1 = lShoulderJoint.getRotationalLimitMotor(1);
		rollMotor1 = lShoulderJoint.getRotationalLimitMotor(2);

		pitchMotor1.setEnableMotor(true);
		yawMotor1.setEnableMotor(true);
		rollMotor1.setEnableMotor(true);

		pitchMotor1.setMaxMotorForce(1);
		yawMotor1.setMaxMotorForce(1);
		rollMotor1.setMaxMotorForce(1);

		pitchMotor2 = rShoulderJoint.getRotationalLimitMotor(0);
		yawMotor2 = rShoulderJoint.getRotationalLimitMotor(1);
		rollMotor2 = rShoulderJoint.getRotationalLimitMotor(2);

		pitchMotor2.setEnableMotor(true);
		yawMotor2.setEnableMotor(true);
		rollMotor2.setEnableMotor(true);

		pitchMotor2.setMaxMotorForce(1);
		yawMotor2.setMaxMotorForce(1);
		rollMotor2.setMaxMotorForce(1);

		join(uArmR, m_shoulders, new Vector3f(0.75f, 1.4f, 0));

		m_agent.attachChild(m_shoulders);
		m_agent.attachChild(uArmL);
		m_agent.attachChild(uArmR);

		rootNode.attachChild(m_agent);

		m_bulletAppState.getPhysicsSpace().addAll(m_agent);

		startLoc = m_agent.getLocalTranslation();
	}

	/// \brief Create a limb
	/// \todo Move this into a separate class
	private Node createLimb (
		float width_, float height_, Vector3f location_,
		boolean rotate_, boolean kinematic_)
	{
		int axis = rotate_ ? PhysicsSpace.AXIS_X : PhysicsSpace.AXIS_Y;
		CapsuleCollisionShape shape =
			new CapsuleCollisionShape (width_, height_, axis);

		Node node = new Node ("Limb");
		RigidBodyControl rigidBodyControl =
			new RigidBodyControl (shape, 1);

		rigidBodyControl.setFriction(0.05f);
/*
		if (kinematic_) {
			rigidBodyControl.setKinematic(true);
		}

		rigidBodyControl.setAngularSleepingThreshold(0);
		rigidBodyControl.setGravity(Vector3f.ZERO);
*/

		node.setLocalTranslation (location_);
		node.addControl (rigidBodyControl);
		node.setShadowMode (ShadowMode.Cast);

		Cylinder limbBox = new Cylinder (32, 32, width_, height_, true);
		Geometry limbGeometry = new Geometry ("Limb", limbBox);

		Material mat2 = new Material(
			assetManager, "Common/MatDefs/Light/Lighting.j3md");

		mat2.setBoolean ("UseMaterialColors",true);
		mat2.setColor ("Ambient", ColorRGBA.Red);
		mat2.setColor ("Diffuse", ColorRGBA.Red);
		mat2.setColor ("Specular", ColorRGBA.White);
		mat2.setFloat ("Shininess", 12);

		limbGeometry.setMaterial (mat2);

		Quaternion newRotation = new Quaternion ();
		Vector3f rotAxis =
			rotate_ ? new Vector3f (0, 1, 0) : new Vector3f (1, 0, 0);

		newRotation.fromAngleAxis (FastMath.PI / 2, rotAxis);
		limbGeometry.setLocalRotation(newRotation);
		node.attachChild(limbGeometry);

		return node;
	}

	/// \brief Build a joint connecting two nodes
	private SixDofJoint join (Node A_, Node B_, Vector3f connectionPoint_)
	{
		Vector3f pivotA =
			A_.worldToLocal (connectionPoint_, new Vector3f());
		Vector3f pivotB =
			B_.worldToLocal (connectionPoint_, new Vector3f());

		return new SixDofJoint(A_.getControl (RigidBodyControl.class),
				       B_.getControl (RigidBodyControl.class),
				       pivotA, pivotB, true);
	}

	@Override
	public void onAction (String name_, boolean isPressed_, float tpf_)
	{
		if (name_.equals("PitchPlus") && isPressed_) {
			//desiredPitchVelocity1 += 1;
		   //System.out.println("PITCH +1: " + desiredPitchVelocity1);
		   //leftShoulderJoint.enableMotor(true, 1, .1f);
		}
		if (name_.equals("PitchMinus") && isPressed_) {
			//desiredPitchVelocity1 -= 1;
			//System.out.println("PITCH -1: " + desiredPitchVelocity1);
			//leftShoulderJoint.enableMotor(true, -1, .1f);
		}
		if (name_.equals("YawPlus") && isPressed_) {
			//desiredYawVelocity1 += 1;
			//System.out.println("YAW +1");
			//leftShoulderJoint.enableMotor(false, 0.0f, 0.1f);
		}
		if (name_.equals("YawMinus") && isPressed_) {
			//desiredYawVelocity1 -= 1;
			//System.out.println("YAW -1");
		}
		if (name_.equals("RollPlus") && isPressed_) {
			//desiredRollVelocity1 += 1;
			//System.out.println("ROLL +1");
		}
		if (name_.equals("RollMinus") && isPressed_) {
			//desiredRollVelocity1 -= 1;
			//System.out.println("ROLL -1");
		}
		if (name_.equals("Reset") && isPressed_) {
			desiredPitchVelocity1 = 0;
			desiredYawVelocity1 = 0;
			desiredRollVelocity1 = 0;
			System.out.println("RESET");
			//leftShoulderJoint.getBodyB().clearForces();
			//leftShoulderJoint.getBodyB().setPhysicsLocation(Vector3f.UNIT_Y);
			//leftShoulderJoint.getBodyB().setPhysicsRotation(new Quaternion());
		}
	}

	class FitnessComparator implements Comparator<Integer>
	{
		Map<Integer, Integer> base;
		public FitnessComparator (Map<Integer, Integer> base_)
		{
			this.base = base_;
		}

		// Note: this comparator imposes orderings that are inconsistent
		// with equals.
		public int compare (Integer a_, Integer b_)
		{
			if (base.get (a_) >= base.get (b_))
			{
				return -1;
			}

			return 1;
		}
	}

	private BulletAppState m_bulletAppState = new BulletAppState();

	private Node m_agent = new Node();
	private Node m_shoulders;

	private SixDofJoint lShoulderJoint;
	private SixDofJoint rShoulderJoint;

	protected RotationalLimitMotor pitchMotor1;
	protected RotationalLimitMotor yawMotor1;
	protected RotationalLimitMotor rollMotor1;

	protected float desiredPitchVelocity1 = 0;
	protected float desiredYawVelocity1 = 0;
	protected float desiredRollVelocity1 = 0;

	protected RotationalLimitMotor pitchMotor2;
	protected RotationalLimitMotor yawMotor2;
	protected RotationalLimitMotor rollMotor2;

	protected float desiredPitchVelocity2 = 0;
	protected float desiredYawVelocity2 = 0;
	protected float desiredRollVelocity2 = 0;

	private Network m_nn;
	private double[] singleLineNetworkInput;

	// Either in evolve mode or evaluate mode. If in evaluate mode, a
	// network id to be evaluated needs to be specified
	private int m_genCounter = 1;
	private int m_runningNetworkId = 1;
	private int m_evalCounter = 0;

	private HashMap<Integer,Integer> fitnessVals =
		new HashMap<Integer,Integer>();
	private TreeMap<Integer,Integer> sortedFitnessVals;

	private Integer[] mostFit;
	private Integer[] leastFit;
	private double survivalPercentage = 0.30;
	private Vector3f startLoc;

	// Constants
	private static final String NET_DIR = "networks/network";
	private static final int POP_SIZE = 20;
	private static final int NUM_GENS = 100;
	private static final int NUM_EVAL_STEPS = 1000;
	private static final int NET_ID_TO_EVAL = 1;
	private static final float SIM_SPEED = 1;

	// Options
	private boolean m_evolveMode = true;
	private boolean m_dvorakMode = false;
	private boolean m_debugMode = true;
}
