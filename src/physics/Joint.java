package physics;

import physics.aether.AetherObject;
import physics.aether.AetherJoint;

import physics.bullet.BulletJoint;
import physics.bullet.DummyBulletSpace;

import physics.jmonkey.MonkeyJoint;

import com.jme3.bullet.PhysicsSpace;

import com.jme3.math.Vector3f;

import com.jme3.scene.Node;


public class Joint implements AbstractJoint, CompositeObject
{
	public Joint ()
	{
		m_joint = new AetherJoint ();
	}

	public Joint (Limb leftJoin_, Limb rightJoin_)
	{
		m_joint = new AetherJoint (leftJoin_, rightJoin_);
	}

	public Joint (Limb leftJoin_, Limb rightJoin_, Vector3f pivot_)
	{
		m_joint = new AetherJoint (leftJoin_, rightJoin_, pivot_);
	}

	public Joint (AbstractJoint joint_)
	{
		m_joint = joint_;
	}

	public Vector3f angularSpeed ()
	{
		return m_joint.angularSpeed ();
	}

	public void setAngularSpeed (Vector3f speed_)
	{
		m_joint.setAngularSpeed (speed_);
	}

	public Limb leftJoin ()
	{
		return m_joint.leftJoin ();
	}

	public Limb rightJoin ()
	{
		return m_joint.rightJoin ();
	}

	public void unregisterFromSpace ()
	{
		m_joint.unregisterFromSpace ();
	}

	public Vector3f position ()
	{
		return m_joint.position ();
	}

	public void registerWithBullet (DummyBulletSpace space_)
	{
		m_joint.unregisterFromSpace ();
		BulletJoint bJoint = new BulletJoint (m_joint);
		bJoint.registerWithBullet (space_);

		m_joint = bJoint;
	}

	public void registerWithJMonkey (PhysicsSpace space_, Node rootNode_)
	{
		m_joint.unregisterFromSpace ();
		MonkeyJoint mJoint = new MonkeyJoint (m_joint);
		mJoint.registerWithJMonkey (space_, rootNode_);

		m_joint = mJoint;
	}

	private AbstractJoint m_joint;
}
