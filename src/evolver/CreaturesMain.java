package evolver;

import network.Network;
import network.NetworkBuilder;

import critter.actuator.Actuator;

import critter.body.Horse;
import critter.body.Tripod;
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
import java.util.List;
import java.util.Map;
import java.util.Random;
import java.util.Set;
import java.util.TreeMap;

import java.util.logging.Level;
import java.util.logging.Logger;
import javax.xml.parsers.DocumentBuilder;
import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.ParserConfigurationException;
import org.w3c.dom.Document;
import org.xml.sax.SAXException;


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

		//Load settings from configuration file
		try
		{
			loadConfigFile ();
		}
		catch (ParserConfigurationException | SAXException ex)
		{
			Logger.getLogger (CreaturesMain.class.getName ()).log (Level.SEVERE, null, ex);
			System.exit (0);
		}

		// Init basic environment
		setupLight ();
		setupKeys (m_dvorakMode);
		flyCam.setMoveSpeed (10);

		// Init physics
		m_bulletAppState = new BulletAppState ();

		m_bulletAppState.setSpeed (simSpeed);

		stateManager.attach (m_bulletAppState);
		PhysicsTestHelper.createPhysicsTestWorld (
			rootNode, assetManager, m_bulletAppState.getPhysicsSpace ());

		//Create initial population of neural network controllers
		m_runningNetworkId = m_evolveMode ? 1 : NET_ID_TO_EVAL;

		if (m_evolveMode)
			initNetworks ();

		initCritter ();
		m_currentActuator = null;

		if (m_debugMode)
			m_bulletAppState.setDebugEnabled (true);

		System.out.println("generation: " + m_genCounter);
		System.out.println("evaluating network: " + m_runningNetworkId);
	}



	private void loadConfigFile () throws ParserConfigurationException, SAXException
	{
		configFile = new File("./config.xml");

		try
		{
			DocumentBuilderFactory docBuilderFactory = DocumentBuilderFactory.newInstance ();
			DocumentBuilder docBuilder = docBuilderFactory.newDocumentBuilder ();
			Document doc = docBuilder.parse (configFile);

			String popSizeString =
				doc.getElementsByTagName ("pop_size").item (0).getTextContent ();
			popSize = Integer.parseInt (popSizeString);

			String numGensString =
				doc.getElementsByTagName ("num_gens").item (0).getTextContent ();
			numGens = Integer.parseInt (numGensString);

			String numEvalStepsString =
				doc.getElementsByTagName ("num_eval_steps").item (0).getTextContent ();
			numEvalSteps = Integer.parseInt (numEvalStepsString);

			String simSpeedString =
				doc.getElementsByTagName ("sim_speed").item (0).getTextContent ();
			simSpeed = Float.parseFloat (simSpeedString);

			String survivalPercentageString =
				doc.getElementsByTagName ("survival_percentage").item (0).getTextContent ();
			survivalPercentage = Double.parseDouble (survivalPercentageString);
		}
		catch (IOException exception)
		{
			System.err.println ("Failed to load config");
			System.exit (0);
		}
	}

	@Override
	public void simpleUpdate(float tpf_)
	{
		if (m_genCounter > numGens)
		{
			return;
		}

		if (m_evalCounter > numEvalSteps)
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
			for (int i = 0; i < (int) simSpeed; i++)
				m_critter.think (tpf_);

			m_critter.act (tpf_);

			m_evalCounter++;
		}
	}

	private void nextNetwork ()
	{
		// Finished the whole population so breed and reset
		if (m_runningNetworkId > popSize)
		{
			System.out.println ("unsorted map: "+fitnessVals);

			FitnessComparator fc = new FitnessComparator (fitnessVals);

			sortedFitnessVals = new TreeMap<Integer,Integer>(fc);
			sortedFitnessVals.putAll(fitnessVals);

			System.out.println("sorted map: "+sortedFitnessVals);
			System.out.println(sortedFitnessVals.firstKey());

			Set sortedKeys = sortedFitnessVals.keySet();

			System.out.println(sortedKeys);

			int numSurviving = (int) (popSize * survivalPercentage);
			int numDying = popSize - numSurviving;
			Object[] sortedArray = sortedKeys.toArray();

			mostFit = Arrays.copyOfRange (sortedArray, 0, numSurviving, Integer[].class);
			leastFit = Arrays.copyOfRange (sortedArray, numSurviving, sortedArray.length,
			                               Integer[].class);

			Repopulator rp = new Repopulator ();

			rp.repopulate (mostFit, leastFit);
			fitnessVals.clear ();

			m_runningNetworkId = 1;
			m_genCounter++;

			if (m_genCounter > numGens)
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
//		BrainVat vat = new NeuralXMLBrainVat (NET_DIR + m_runningNetworkId + ".xml",
//		                                      NeuralPushPullBrain.class);
		BrainVat vat = new NullBrainVat ();
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
		inputManager.addMapping("Highlight", new KeyTrigger(KeyInput.KEY_J));
		inputManager.addMapping("1", new KeyTrigger(KeyInput.KEY_1));
		inputManager.addMapping("2", new KeyTrigger(KeyInput.KEY_2));
		inputManager.addMapping("3", new KeyTrigger(KeyInput.KEY_3));
		inputManager.addMapping("4", new KeyTrigger(KeyInput.KEY_4));
		inputManager.addMapping("5", new KeyTrigger(KeyInput.KEY_5));
		inputManager.addMapping("6", new KeyTrigger(KeyInput.KEY_6));

		inputManager.addListener(this, "Highlight", "1", "2", "3", "4", "5", "6");
	}

	/// \brief Set up the input key mappings for a dvorak keyboard
	private void setupDvorakKeys ()
	{
		inputManager.addMapping("Highlight", new KeyTrigger(KeyInput.KEY_J));
		inputManager.addMapping("1", new KeyTrigger(KeyInput.KEY_1));
		inputManager.addMapping("2", new KeyTrigger(KeyInput.KEY_2));
		inputManager.addMapping("3", new KeyTrigger(KeyInput.KEY_3));
		inputManager.addMapping("4", new KeyTrigger(KeyInput.KEY_4));
		inputManager.addMapping("5", new KeyTrigger(KeyInput.KEY_5));
		inputManager.addMapping("6", new KeyTrigger(KeyInput.KEY_6));

		inputManager.addListener(this, "Highlight", "1", "2", "3", "4", "5", "6");
	}

	/// \brief Initialize a population of networks
	private void initNetworks ()
	{
		for (int i = 0; i < popSize; i++)
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
		if (name_.equals("Highlight") && isPressed_)
		{
			if (m_currentActuator == null)
			{
				m_currentActuator = m_critter.controls ().first ();
			}
			else
			{
				m_currentActuator.unHighlight ();
				m_currentActuator = m_critter.controls ().next (m_currentActuator);
			}

			if (m_currentActuator == null)
				return;

			m_currentActuator.highlight ();
		}
		else if (name_.equals ("1") || name_.equals ("2") || name_.equals ("3") ||
		         name_.equals ("4") || name_.equals ("5") || name_.equals ("6"))
		{
			if (m_currentActuator == null)
				return;

			float val = isPressed_ ? 1f : 0f;

			List<Float> activation = new ArrayList<Float> ();

			for (int i = 1; i < m_currentActuator.size () + 1; i++)
			{
				if (i == Integer.parseInt (name_))
					activation.add (val);
				else if (i + m_currentActuator.size () == Integer.parseInt (name_))
					activation.add (-val);
				else
					activation.add (0f);
			}

			m_currentActuator.act (activation, tpf_);
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

	private static File configFile;

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
	private Actuator m_currentActuator;
	private Vector3f m_startLocation;

	// Constants
	private static final String NET_DIR = "networks/network";
	private static int popSize = 10;
	private static int numGens = 100;
	private static int numEvalSteps = 1000;
	private static final int NET_ID_TO_EVAL = 1;
	private static float simSpeed = 1;

	// Options
	private boolean m_evolveMode = true;
	private boolean m_dvorakMode = false;
	private boolean m_debugMode = true;
}
