package physics.jmonkey;

import physics.AbstractJoint;
import physics.Limb;

import com.jme3.asset.DesktopAssetManager;

import com.jme3.bullet.PhysicsSpace;

import com.jme3.bullet.control.RigidBodyControl;

import com.jme3.bullet.joints.SixDofJoint;
import com.jme3.bullet.joints.motors.RotationalLimitMotor;
import com.jme3.bullet.joints.motors.TranslationalLimitMotor;

import com.jme3.material.Material;

import com.jme3.math.ColorRGBA;
import com.jme3.math.Vector3f;

import com.jme3.renderer.queue.RenderQueue.ShadowMode;

import com.jme3.scene.shape.Sphere;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;


public class MonkeyJoint implements AbstractJoint, MonkeyObject
{
	private static final int AXIS_SAMPLES = 32;
	private static final int RADIAL_SAMPLES = 32;
	private static final float ELBOW_RADIUS = 0.25f;
	private static final String ELBOW_NAME = "elbow";

	public MonkeyJoint (AbstractJoint joint_)
	{
		m_leftJoin = joint_.leftJoin ();
		m_rightJoin = joint_.rightJoin ();

		m_pivot = joint_.position ();
		m_localPivot = joint_.position ();

		m_space = null;
		m_highlight = null;

		if (joint_.isHighlighted ())
			highlight ();
	}

	public Vector3f angularSpeed ()
	{
		return new Vector3f (m_joint.getRotationalLimitMotor (0).getTargetVelocity (),
		                     m_joint.getRotationalLimitMotor (1).getTargetVelocity (),
		                     m_joint.getRotationalLimitMotor (2).getTargetVelocity ());
	}

	public void setAngularSpeed (Vector3f speed_)
	{
		m_joint.getRotationalLimitMotor (0).setTargetVelocity (speed_.getX ());
		m_joint.getRotationalLimitMotor (1).setTargetVelocity (speed_.getY ());
		m_joint.getRotationalLimitMotor (2).setTargetVelocity (speed_.getZ ());
	}

	public Limb leftJoin ()
	{
		return m_leftJoin;
	}

	public Limb rightJoin ()
	{
		return m_rightJoin;
	}

	public void highlight ()
	{
		Sphere ball = new Sphere (AXIS_SAMPLES, RADIAL_SAMPLES, ELBOW_RADIUS);
		m_highlight = new Geometry (ELBOW_NAME, ball);

		DesktopAssetManager manager = new DesktopAssetManager (true);
		Material material = new Material(manager, "Common/MatDefs/Light/Lighting.j3md");

		material.setBoolean ("UseMaterialColors",true);
		material.setColor ("Ambient", ColorRGBA.Yellow);
		material.setColor ("Diffuse", ColorRGBA.Yellow);
		material.setColor ("Specular", ColorRGBA.White);
		material.setFloat ("Shininess", 12);

		m_highlight.setMaterial (material);

		if (m_space == null)
			return;

		m_highlight.setLocalTranslation (m_localPivot);
		m_leftJoin.monkeyNode ().attachChild (m_highlight);
	}

	public void unHighlight ()
	{
		m_highlight.removeFromParent ();

		m_highlight = null;
	}

	public boolean isHighlighted ()
	{
		return m_highlight != null;
	}

	public void unregisterFromSpace ()
	{
		m_space.remove (m_joint);
		m_space = null;
	}

	public Vector3f position ()
	{
		return m_pivot;
	}

	public void registerWithJMonkey (PhysicsSpace space_, Node rootNode_)
	{
		Node leftNode = m_leftJoin.monkeyNode ();
		Node rightNode = m_rightJoin.monkeyNode ();

		Vector3f leftPivot = leftNode.worldToLocal (m_pivot, new Vector3f());
		Vector3f rightPivot = rightNode.worldToLocal (m_pivot, new Vector3f());

		m_localPivot = leftPivot;

		m_joint = new SixDofJoint (leftNode.getControl (RigidBodyControl.class),
		                           rightNode.getControl (RigidBodyControl.class),
		                           leftPivot, rightPivot, true);

		m_joint.getRotationalLimitMotor (0).setEnableMotor (true);
		m_joint.getRotationalLimitMotor (1).setEnableMotor (true);
		m_joint.getRotationalLimitMotor (2).setEnableMotor (true);

		m_joint.getRotationalLimitMotor (0).setMaxMotorForce (1);
		m_joint.getRotationalLimitMotor (1).setMaxMotorForce (1);
		m_joint.getRotationalLimitMotor (2).setMaxMotorForce (1);

		if (isHighlighted ())
		{
			m_highlight.setLocalTranslation (leftPivot);
			leftNode.attachChild (m_highlight);
		}

		m_space = space_;
		m_space.add (m_joint);
	}

	private PhysicsSpace m_space;
	private SixDofJoint m_joint;

	private Limb m_leftJoin;
	private Limb m_rightJoin;

	Geometry m_highlight;

	private Vector3f m_pivot;
	private Vector3f m_localPivot;
}
