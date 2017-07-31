package physics.aether;

import physics.AbstractJoint;
import physics.Limb;

import utilities.LineUtils;

import com.jme3.math.Vector3f;


public class AetherJoint implements AbstractJoint, AetherObject
{
	public AetherJoint ()
	{
		m_speed = new Vector3f (0, 0, 0);
		m_pivot = new Vector3f (0, 0, 0);

		m_leftJoin = new Limb ();
		m_rightJoin = new Limb ();
	}

	public AetherJoint (Limb leftJoin_, Limb rightJoin_, Vector3f pivot_)
	{
		m_speed = new Vector3f (0, 0, 0);
		m_pivot = pivot_;

		m_leftJoin = leftJoin_;
		m_rightJoin = rightJoin_;
	}

	public AetherJoint (Limb leftJoin_, Limb rightJoin_)
	{
		m_speed = new Vector3f (0, 0, 0);

		LineUtils.Line leftLine = new LineUtils.Line (leftJoin_.position (),
		                                              leftJoin_.alignment ());
		LineUtils.Line rightLine = new LineUtils.Line (rightJoin_.position (),
		                                               rightJoin_.alignment ());

		m_pivot = LineUtils.closestPoint (leftLine, rightLine);

		m_leftJoin = leftJoin_;
		m_rightJoin = rightJoin_;
	}

	public AetherJoint (AbstractJoint joint_)
	{
		m_speed = joint_.angularSpeed ();
		m_pivot = joint_.position ();

		m_leftJoin = joint_.leftJoin ();
		m_rightJoin = joint_.rightJoin ();
	}

	public Vector3f angularSpeed ()
	{
		return m_speed;
	}

	public void setAngularSpeed (Vector3f speed_)
	{
		m_speed = speed_;
	}

	public Limb leftJoin ()
	{
		return m_leftJoin;
	}

	public Limb rightJoin ()
	{
		return m_rightJoin;
	}

	public Vector3f position ()
	{
		return m_pivot;
	}

	public void unregisterFromSpace ()
	{
	}

	///////////////////////////////////////////////////////////////////////////
	private Vector3f m_speed;
	private Vector3f m_pivot;

	private Limb m_leftJoin;
	private Limb m_rightJoin;
}
