package evolver;

import critter.body.Horse;
import critter.body.SimpleBody;
import critter.body.Body;
import critter.Critter;
import critter.brain.NeuralRandomBrainVat;
import critter.brain.NullBrainVat;
import critter.brain.BrainVat;
import critter.brain.NeuralPushPullBrain;
import critter.BirthingPod;
import critter.SimpleWalkerBirthingPod;
import critter.HorseBirthingPod;
import critter.TripodBirthingPod;
import critter.SnakeBirthingPod;

import physics.aether.AetherLimb;
import physics.jmonkey.MonkeyLimb;
import physics.Limb;
import physics.Joint;

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


public class DemoMain extends SimpleApplication implements ActionListener
{
	/// \brief Main function. Just starts the app.
	public static void main (String[] args)
	{
		DemoMain app = new DemoMain();

		app.setPauseOnLostFocus(false);
		app.start();

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

		if (m_debugMode)
			m_bulletAppState.setDebugEnabled (true);

		BrainVat vat = new NeuralRandomBrainVat ("networks/test.xml", 1,
		                                         NeuralPushPullBrain.class);
//		BrainVat vat = new NullBrainVat ();
//		BirthingPod pod = new SimpleWalkerBirthingPod (vat);
		BirthingPod pod = new SnakeBirthingPod (vat);

		m_critter = pod.birth ();
		m_critter.body ().registerWithJMonkey (m_bulletAppState.getPhysicsSpace (), rootNode);
	}


	@Override
	public void simpleUpdate(float tpf_)
	{
		m_critter.think (tpf_);
		m_critter.act (tpf_);
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

	private BulletAppState m_bulletAppState = new BulletAppState();
	private Critter m_critter;

	// Constants
	private static final float SIM_SPEED = 1;

	// Options
	private boolean m_dvorakMode = false;
	private boolean m_debugMode = true;
}
