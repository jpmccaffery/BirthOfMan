package physics.jmonkey;

import physics.AbstractJoint;
import physics.Limb;

import utilities.CoordUtils;

import com.bulletphysics.dynamics.constraintsolver.Generic6DofConstraint;
import com.bulletphysics.dynamics.constraintsolver.TypedConstraint;
import com.bulletphysics.dynamics.constraintsolver.TypedConstraintType;

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
import com.jme3.scene.shape.Cylinder;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;


public class MonkeyJoint implements AbstractJoint, MonkeyObject
{
	private static final int AXIS_SAMPLES = 32;
	private static final int RADIAL_SAMPLES = 32;
	private static final float ELBOW_RADIUS = 0.25f;
	private static final String ELBOW_NAME = "elbow";

	private static final float SPOKE_LEN = 2f;
	private static final float SPOKE_RAD = 0.05f;

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

		Cylinder xSpoke = new Cylinder (AXIS_SAMPLES, RADIAL_SAMPLES,
		                                SPOKE_RAD, SPOKE_LEN, true);
		m_xGeom = new Geometry ("Spoke", xSpoke);

		Cylinder ySpoke = new Cylinder (AXIS_SAMPLES, RADIAL_SAMPLES,
		                                SPOKE_RAD, SPOKE_LEN, true);
		m_yGeom = new Geometry ("Spoke", ySpoke);

		Cylinder zSpoke = new Cylinder (AXIS_SAMPLES, RADIAL_SAMPLES,
		                                SPOKE_RAD, SPOKE_LEN, true);
		m_zGeom = new Geometry ("Spoke", zSpoke);

		Material spokeMaterial = new Material(manager, "Common/MatDefs/Light/Lighting.j3md");

		spokeMaterial.setBoolean ("UseMaterialColors",true);
		spokeMaterial.setColor ("Ambient", ColorRGBA.Green);
		spokeMaterial.setColor ("Diffuse", ColorRGBA.Green);
		spokeMaterial.setColor ("Specular", ColorRGBA.White);
		spokeMaterial.setFloat ("Shininess", 12);

		m_xGeom.setMaterial (spokeMaterial);
		m_yGeom.setMaterial (spokeMaterial);
		m_zGeom.setMaterial (spokeMaterial);

		if (m_space == null)
			return;

		m_leftJoin.monkeyNode ().getParent ().attachChild (m_highlight);
		m_leftJoin.monkeyNode ().getParent ().attachChild (m_xGeom);
		m_leftJoin.monkeyNode ().getParent ().attachChild (m_yGeom);
		m_leftJoin.monkeyNode ().getParent ().attachChild (m_zGeom);

		updateHighlight ();
	}

	public void updateHighlight ()
	{
		if (! isHighlighted () || m_space == null)
			return;

		m_highlight.setLocalTranslation (position ());

		Generic6DofConstraint bulletJoint = (Generic6DofConstraint) m_joint.getObjectId ();

		bulletJoint.buildJacobian ();

		javax.vecmath.Vector3f bulletAxis = new javax.vecmath.Vector3f ();
		Vector3f monkeyAxis = new Vector3f ();

		bulletJoint.getAxis (0, bulletAxis);
		monkeyAxis = new Vector3f (bulletAxis.x, bulletAxis.y, bulletAxis.z);
		monkeyAxis.normalizeLocal ();
		m_xGeom.setLocalTranslation (position ().add (monkeyAxis.mult (SPOKE_LEN / 2)));
		m_xGeom.setLocalRotation (CoordUtils.rotationFromZAxis (monkeyAxis));

		bulletJoint.getAxis (1, bulletAxis);
		monkeyAxis = new Vector3f (bulletAxis.x, bulletAxis.y, bulletAxis.z);
		monkeyAxis.normalizeLocal ();
		m_yGeom.setLocalTranslation (position ().add (monkeyAxis.mult (SPOKE_LEN / 2)));
		m_yGeom.setLocalRotation (CoordUtils.rotationFromZAxis (monkeyAxis));

		bulletJoint.getAxis (2, bulletAxis);
		monkeyAxis = new Vector3f (bulletAxis.x, bulletAxis.y, bulletAxis.z);
		monkeyAxis.normalizeLocal ();
		m_zGeom.setLocalTranslation (position ().add (monkeyAxis.mult (SPOKE_LEN / 2)));
		m_zGeom.setLocalRotation (CoordUtils.rotationFromZAxis (monkeyAxis));
	}

	public void unHighlight ()
	{
		m_highlight.removeFromParent ();
		m_xGeom.removeFromParent ();
		m_yGeom.removeFromParent ();
		m_zGeom.removeFromParent ();

		m_highlight = null;
		m_xGeom = null;
		m_yGeom = null;
		m_zGeom = null;
	}

	public boolean isHighlighted ()
	{
		return m_highlight != null;
	}

	public void unregisterFromSpace ()
	{
		unHighlight ();

		m_space.remove (m_joint);
		m_space = null;
	}

	public Vector3f position ()
	{
		if (m_space == null)
			return m_pivot;

		Node leftNode = m_leftJoin.monkeyNode ();

		return leftNode.localToWorld (m_localPivot, new Vector3f());
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

		m_joint.setAngularUpperLimit (new Vector3f (1000f, 1000f, 1000f));
		m_joint.setAngularLowerLimit (new Vector3f (-1000f, -1000f, -1000f));

		if (isHighlighted ())
		{
			leftNode.getParent ().attachChild (m_highlight);
			leftNode.getParent ().attachChild (m_xGeom);
			leftNode.getParent ().attachChild (m_yGeom);
			leftNode.getParent ().attachChild (m_zGeom);

			updateHighlight ();
		}

		m_space = space_;
		m_space.add (m_joint);
	}

	private PhysicsSpace m_space;
	private SixDofJoint m_joint;

	private Limb m_leftJoin;
	private Limb m_rightJoin;

	Geometry m_highlight;
	Geometry m_xGeom;
	Geometry m_yGeom;
	Geometry m_zGeom;

	private Vector3f m_pivot;
	private Vector3f m_localPivot;
}
