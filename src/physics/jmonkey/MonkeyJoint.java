package physics.jmonkey;

import physics.AbstractJoint;
import physics.Limb;
import physics.RenderUpdater;

import utilities.CoordUtils;

import com.bulletphysics.dynamics.constraintsolver.Generic6DofConstraint;
import com.bulletphysics.dynamics.constraintsolver.TypedConstraint;
import com.bulletphysics.dynamics.constraintsolver.TypedConstraintType;

import com.bulletphysics.linearmath.Transform;

import com.jme3.asset.DesktopAssetManager;

import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.PhysicsTickListener;

import com.jme3.bullet.control.RigidBodyControl;

import com.jme3.bullet.joints.SixDofJoint;
import com.jme3.bullet.joints.motors.RotationalLimitMotor;
import com.jme3.bullet.joints.motors.TranslationalLimitMotor;

import com.jme3.material.Material;

import com.jme3.math.ColorRGBA;
import com.jme3.math.Matrix3f;
import com.jme3.math.Vector3f;

import com.jme3.renderer.queue.RenderQueue.ShadowMode;

import com.jme3.scene.shape.Sphere;
import com.jme3.scene.shape.Cylinder;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;


public class MonkeyJoint implements AbstractJoint, MonkeyObject, PhysicsTickListener
{
	private static final int AXIS_SAMPLES = 32;
	private static final int RADIAL_SAMPLES = 32;
	private static final float ELBOW_RADIUS = 0.25f;
	private static final String ELBOW_NAME = "elbow";

	private static final float SPOKE_LEN = 2f;
	private static final float SPOKE_RAD = 0.05f;

	private static final Material YELLOW;
	private static final Material GREEN;

	private static final float PI = (float) Math.PI;
	private static final float PI_2 = (float) Math.PI / 2f;
	private static final float PI_4 = (float) Math.PI / 4f;
	private static final float EPSILON = 0.001f;

	static
	{
		DesktopAssetManager manager = new DesktopAssetManager (true);

		YELLOW = new Material(manager, "Common/MatDefs/Light/Lighting.j3md");

		YELLOW.setBoolean ("UseMaterialColors",true);
		YELLOW.setColor ("Ambient", ColorRGBA.Yellow);
		YELLOW.setColor ("Diffuse", ColorRGBA.Yellow);
		YELLOW.setColor ("Specular", ColorRGBA.White);
		YELLOW.setFloat ("Shininess", 12);

		GREEN = new Material(manager, "Common/MatDefs/Light/Lighting.j3md");

		GREEN.setBoolean ("UseMaterialColors",true);
		GREEN.setColor ("Ambient", ColorRGBA.Green);
		GREEN.setColor ("Diffuse", ColorRGBA.Green);
		GREEN.setColor ("Specular", ColorRGBA.White);
		GREEN.setFloat ("Shininess", 12);
	}


	public MonkeyJoint (AbstractJoint joint_)
	{
		m_leftJoin = joint_.leftJoin ();
		m_rightJoin = joint_.rightJoin ();

		m_pivot = joint_.position ();
//		m_localPivot = joint_.position ();
		m_localPivot = null;

		m_angleOffset2 = 0;
		m_angleOffset3 = 0;
		m_zTarget = 1000f;

		m_space = null;
		m_highlight = null;
		m_highlight = null;
		m_highlight = null;
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
		if (m_space == null)
			return;

		Sphere ball = new Sphere (AXIS_SAMPLES, RADIAL_SAMPLES, ELBOW_RADIUS);
		m_highlight = new Geometry (ELBOW_NAME, ball);

		m_highlight.setMaterial (YELLOW);

		Cylinder xCylinder = new Cylinder (AXIS_SAMPLES, RADIAL_SAMPLES,
		                                   SPOKE_RAD, SPOKE_LEN, true);
		m_xSpoke = new Geometry ("Spoke", xCylinder);

		Cylinder yCylinder = new Cylinder (AXIS_SAMPLES, RADIAL_SAMPLES,
		                                   SPOKE_RAD, SPOKE_LEN, true);
		m_ySpoke = new Geometry ("Spoke", yCylinder);

		Cylinder zCylinder = new Cylinder (AXIS_SAMPLES, RADIAL_SAMPLES,
		                                   SPOKE_RAD, SPOKE_LEN, true);
		m_zSpoke = new Geometry ("Spoke", zCylinder);

		m_xSpoke.setMaterial (GREEN);
		m_ySpoke.setMaterial (GREEN);
		m_zSpoke.setMaterial (GREEN);

		m_leftJoin.monkeyNode ().getParent ().attachChild (m_highlight);
		m_leftJoin.monkeyNode ().getParent ().attachChild (m_xSpoke);
		m_leftJoin.monkeyNode ().getParent ().attachChild (m_ySpoke);
		m_leftJoin.monkeyNode ().getParent ().attachChild (m_zSpoke);
	}

	public void unHighlight ()
	{
		m_highlight.removeFromParent ();
		m_xSpoke.removeFromParent ();
		m_ySpoke.removeFromParent ();
		m_zSpoke.removeFromParent ();

		m_highlight = null;
		m_xSpoke = null;
		m_ySpoke = null;
		m_zSpoke = null;
	}

	public boolean isHighlighted ()
	{
		return m_highlight != null;
	}

	public void unregisterFromSpace ()
	{
		if (m_space == null)
			return;

		m_localPivot = null;

		unHighlight ();
		destroyJoint ();

		m_space.removeTickListener (this);
		m_space = null;

		m_updater.unregisterUpdatable (this);
		m_updater = null;
	}

	private void destroyJoint ()
	{
		m_space.remove (m_joint);
		m_joint = null;
	}

	public Vector3f position ()
	{
		if (m_space == null)
			return m_pivot;

		Node leftNode = m_leftJoin.monkeyNode ();

		return leftNode.localToWorld (m_localPivot, new Vector3f());
	}

	public void registerWithJMonkey (PhysicsSpace space_, RenderUpdater updater_)
	{
		if (isHighlighted ())
		{
			Node leftNode = m_leftJoin.monkeyNode ();

			leftNode.getParent ().attachChild (m_highlight);
			leftNode.getParent ().attachChild (m_xSpoke);
			leftNode.getParent ().attachChild (m_ySpoke);
			leftNode.getParent ().attachChild (m_zSpoke);
		}

		m_space = space_;
		m_space.addTickListener (this);

		m_updater = updater_;
		m_updater.registerUpdatable (this);

		buildJoint ();
	}

	private void buildJoint ()
	{
		Node leftNode = m_leftJoin.monkeyNode ();
		Node rightNode = m_rightJoin.monkeyNode ();

		if (m_localPivot == null)
			m_localPivot = leftNode.worldToLocal (m_pivot, new Vector3f());

		Vector3f leftPivot = m_localPivot;
		Vector3f worldPivot = leftNode.localToWorld (m_localPivot, new Vector3f ());
		Vector3f rightPivot = rightNode.worldToLocal (worldPivot, new Vector3f());

		// Add rotational offset
		Matrix3f id = new Matrix3f (1f, 0, 0, 0, 1f, 0, 0, 0, 1f);
		Matrix3f mat2 = new Matrix3f ();
		Matrix3f mat3 = new Matrix3f ();

//		Vector3f localY = leftNode.worldToLocal (new Vector3f (0, 1f, 0), new Vector3f ());
//		Vector3f localZ = leftNode.worldToLocal (new Vector3f (0, 0, 1f), new Vector3f ());
		Vector3f localY = new Vector3f (0, 1f, 0);
		Vector3f localZ = new Vector3f (0, 0, 1f);

		mat2.fromAngleAxis (m_angleOffset2 * PI_2, localY);
		mat3.fromAngleAxis (m_angleOffset3 * PI_2, localZ);

		m_joint = new SixDofJoint (leftNode.getControl (RigidBodyControl.class),
		                           rightNode.getControl (RigidBodyControl.class),
		                           leftPivot, rightPivot, mat3.mult (mat2), id, true);

		m_joint.getRotationalLimitMotor (0).setEnableMotor (true);
		m_joint.getRotationalLimitMotor (1).setEnableMotor (true);
		m_joint.getRotationalLimitMotor (2).setEnableMotor (true);

		m_joint.getRotationalLimitMotor (0).setMaxMotorForce (1);
		m_joint.getRotationalLimitMotor (1).setMaxMotorForce (1);
		m_joint.getRotationalLimitMotor (2).setMaxMotorForce (1);

		m_joint.setAngularUpperLimit (new Vector3f (1000f, 1000f, 1000f));
		m_joint.setAngularLowerLimit (new Vector3f (-1000f, -1000f, -1000f));

		m_space.add (m_joint);
	}

	public void physicsTick (PhysicsSpace space_, float tpf_)
	{
		if (m_joint == null)
			return;

		updateHighlights ();

		boolean needsUpdate = false;
		Generic6DofConstraint bulletJoint = (Generic6DofConstraint) m_joint.getObjectId ();

		// Check max values
		if (m_zTarget == 1000f)
		{
			needsUpdate = true;
			m_zTarget = bulletJoint.getAngle (2);
		}

		// Check Joints are in bound
		if (bulletJoint.getAngle (1) > 0.9f * PI_2)
		{
			needsUpdate = true;
			m_angleOffset2 -= 1;

			if (m_angleOffset2 == -4)
				m_angleOffset2 = 0;
		}
		else if (bulletJoint.getAngle (1) < -0.9f * PI_2)
		{
			needsUpdate = true;
			m_angleOffset2 += 1;

			if (m_angleOffset2 == 4)
				m_angleOffset2 = 0;
		}
		if (bulletJoint.getAngle (2) > 0.9f * PI_2)
		{
			needsUpdate = true;
			m_angleOffset3 -= 1;
			m_zTarget -= PI_2;

			if (m_angleOffset3 == -4)
			{
				System.err.println ("Z rotation should be locked");
//				System.exit (0);
				m_angleOffset3 = 0;
			}
		}
		else if (bulletJoint.getAngle (2) < -0.9f * PI_2)
		{
			needsUpdate = true;
			m_angleOffset3 += 1;
			m_zTarget += PI_2;

			if (m_angleOffset3 == 4)
			{
				System.err.println ("Z rotation should be locked");
//				System.exit (0);
				m_angleOffset3 = 0;
			}
		}

		if (needsUpdate)
		{
			Vector3f speed = angularSpeed ();

			destroyJoint ();
			buildJoint ();

			setAngularSpeed (speed);

			m_joint.setAngularUpperLimit (new Vector3f (1000f, 1000f, m_zTarget + EPSILON));
			m_joint.setAngularLowerLimit (new Vector3f (-1000f, -1000f, m_zTarget - EPSILON));
		}
	}

	private void updateHighlights ()
	{
		if (! isHighlighted () || m_space == null)
			return;

		Generic6DofConstraint bulletJoint = (Generic6DofConstraint) m_joint.getObjectId ();

		// Display state information
		System.out.println ("------------------------------------------");
		System.out.println ("X-angle- " + bulletJoint.getAngle (0) * 180 / 3.1415);
		System.out.println ("Y-angle- " + bulletJoint.getAngle (1) * 180 / 3.1415);
		System.out.println ("Z-angle- " + bulletJoint.getAngle (2) * 180 / 3.1415);

		System.out.println ("X-axis- " +
		                    bulletJoint.getAxis (0, new javax.vecmath.Vector3f ()));
		System.out.println ("Y-axis- " +
		                    bulletJoint.getAxis (1, new javax.vecmath.Vector3f ()));
		System.out.println ("Z-axis- " +
		                    bulletJoint.getAxis (2, new javax.vecmath.Vector3f ()));

		// Position and rotate decorations
		javax.vecmath.Vector3f bulletAxis = new javax.vecmath.Vector3f ();

		bulletJoint.getAxis (0, bulletAxis);
		m_rotationAxis1 = new Vector3f (bulletAxis.x, bulletAxis.y, bulletAxis.z);
		m_rotationAxis1.normalizeLocal ();

		bulletJoint.getAxis (1, bulletAxis);
		m_rotationAxis2 = new Vector3f (bulletAxis.x, bulletAxis.y, bulletAxis.z);
		m_rotationAxis2.normalizeLocal ();

		bulletJoint.getAxis (2, bulletAxis);
		m_rotationAxis3 = new Vector3f (bulletAxis.x, bulletAxis.y, bulletAxis.z);
		m_rotationAxis3.normalizeLocal ();
	}

	public void renderUpdate (float tpf_)
	{
		if (! isHighlighted () || m_space == null || m_rotationAxis1 == null)
			return;

		// Position and rotate decorations
		m_highlight.setLocalTranslation (position ());

		m_xSpoke.setLocalTranslation (position ().add (m_rotationAxis1.mult (SPOKE_LEN / 2)));
		m_xSpoke.setLocalRotation (CoordUtils.rotationFromZAxis (m_rotationAxis1));

		m_ySpoke.setLocalTranslation (position ().add (m_rotationAxis2.mult (SPOKE_LEN / 2)));
		m_ySpoke.setLocalRotation (CoordUtils.rotationFromZAxis (m_rotationAxis2));

		m_zSpoke.setLocalTranslation (position ().add (m_rotationAxis3.mult (SPOKE_LEN / 2)));
		m_zSpoke.setLocalRotation (CoordUtils.rotationFromZAxis (m_rotationAxis3));
	}

	public void prePhysicsTick (PhysicsSpace space_, float tpf_)
	{
	}

	private PhysicsSpace m_space;
	private RenderUpdater m_updater;
	private SixDofJoint m_joint;

	private Limb m_leftJoin;
	private Limb m_rightJoin;

	private Geometry m_highlight;
	private Geometry m_xSpoke;
	private Geometry m_ySpoke;
	private Geometry m_zSpoke;

	private Vector3f m_pivot;
	private Vector3f m_localPivot;

	private Vector3f m_rotationAxis1;
	private Vector3f m_rotationAxis2;
	private Vector3f m_rotationAxis3;

	// Angle 1 isn't restricted so no offset required
	private int m_angleOffset2;
	private int m_angleOffset3;
	private float m_zTarget;
}
