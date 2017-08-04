package evolver;

import network.Network;
import network.NetworkBuilder;

import critter.body.Horse;
import critter.body.Tripod;
import critter.body.Snake;
import critter.body.SimpleBody;
import critter.body.Body;

import critter.brain.NeuralPushPullBrain;
import critter.brain.NeuralRandomBrainVat;
import critter.brain.NeuralXMLBrainVat;
import critter.brain.NullBrainVat;
import critter.brain.BrainVat;

import critter.Critter;
import critter.BirthingPod;
import critter.WalkerBirthingPod;

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
	public static void main (String[] args_)
	{
		CreaturesMain app = new CreaturesMain ();

		app.setPauseOnLostFocus (false);
		app.start ();
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

		//Create initial population of neural network controllers
		m_runningNetworkId = m_evolveMode ? 1 : NET_ID_TO_EVAL;

		if (m_evolveMode)
			initNetworks ();

		initCritter ();

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

			Vector3f endLocation = m_critter.body ().position ();
			float distanceTraveled = m_startLocation.distance (endLocation);

			distanceTraveled = Math.round(distanceTraveled * 100);
			tempFitness = (int) distanceTraveled;
			fitnessVals.put (m_runningNetworkId, tempFitness);
			m_evalCounter = 0;
			m_runningNetworkId++;

			nextNetwork ();
		}
		else
		{
			for (int i = 0; i < (int) SIM_SPEED; i++)
				m_critter.think (tpf_);

			m_critter.act (tpf_);

			m_evalCounter++;
		}
	}

	private void nextNetwork ()
	{
		// Finished the whole population so breed and reset
		if (m_runningNetworkId > POP_SIZE)
		{
			System.out.println ("unsorted map: "+fitnessVals);

			FitnessComparator fc = new FitnessComparator (fitnessVals);

			sortedFitnessVals = new TreeMap<Integer,Integer>(fc);
			sortedFitnessVals.putAll(fitnessVals);

			System.out.println("sorted map: "+sortedFitnessVals);
			System.out.println(sortedFitnessVals.firstKey());

			Set sortedKeys = sortedFitnessVals.keySet();

			System.out.println(sortedKeys);

			int numSurviving = (int) (POP_SIZE * survivalPercentage);
			int numDying = POP_SIZE - numSurviving;
			Object[] sortedArray = sortedKeys.toArray();

			mostFit = Arrays.copyOfRange (sortedArray, 0, numSurviving, Integer[].class);
			leastFit = Arrays.copyOfRange (sortedArray, numSurviving, sortedArray.length,
			                               Integer[].class);

			Repopulator rp = new Repopulator ();

			rp.repopulate (mostFit, leastFit);
			fitnessVals.clear ();

			m_runningNetworkId = 1;
			m_genCounter++;

			if (m_genCounter > NUM_GENS)
			{
				System.out.println ("Finished!");
				System.exit (0);
			}
			else
			{
				System.out.println("generation: " + m_genCounter);
			}
		}

		// Otherwise load the next network
		System.out.println ("evaluating network: " + m_runningNetworkId);

		m_critter.body ().unregisterFromSpace ();
		initCritter ();
	}

	private void initCritter ()
	{
		Body body = new SimpleBody ();
		BrainVat vat = new NeuralXMLBrainVat (NET_DIR + m_runningNetworkId + ".xml",
		                                      NeuralPushPullBrain.class);
		BirthingPod pod = new WalkerBirthingPod (vat, body);

		m_critter = pod.birth ();
		m_critter.body ().registerWithJMonkey (m_bulletAppState.getPhysicsSpace (), rootNode);
		m_startLocation = m_critter.body ().position ();
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
	private void initNetworks ()
	{
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

	private Critter m_critter;
	private Vector3f m_startLocation;

	// Constants
	private static final String NET_DIR = "networks/network";
	private static final int POP_SIZE = 20;
	private static final int NUM_GENS = 100;
	private static final int NUM_EVAL_STEPS = 1000;
	private static final int NET_ID_TO_EVAL = 1;
	private static final float SIM_SPEED = 100;

	// Options
	private boolean m_evolveMode = true;
	private boolean m_dvorakMode = false;
	private boolean m_debugMode = true;
}
