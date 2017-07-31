package physics.bullet;

import physics.AbstractLimb;
import physics.Limb;

import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;


public class BulletLimb implements AbstractLimb, BulletObject
{
	public BulletLimb (AbstractLimb limb_)
	{
/*
		m_speed = limb_.speed ();
		m_length = limb_.length ();
		m_mass = limb_.mass ();

		m_position = limb_.position ();
*/
	}

	public Vector3f speed ()
	{
		return new Vector3f (0f, 0f, 0f);
	}

	public void setSpeed (Vector3f speed_)
	{
	}

	public float length ()
	{
		return 0;
	}

	public void setLength (float length_)
	{
	}

	public float mass ()
	{
		return 0;
	}

	public void setMass (float mass_)
	{
	}

	public Quaternion orientation ()
	{
		return new Quaternion (0f, 0f, 0f, 0f);
	}

	public void setOrientation (Quaternion orientation_)
	{
	}

	public Vector3f alignment ()
	{
		return new Vector3f (0f, 0f, 0f);
	}

	public void setAlignment (Vector3f alignment_)
	{
	}

	public int numCaps ()
	{
		return 0;
	}

	public Vector3f position ()
	{
		return new Vector3f (0f, 0f, 0f);
	}

	public void setPosition (Vector3f position_)
	{
	}

	public void unregisterFromSpace ()
	{
	}

	public void registerWithBullet (DummyBulletSpace space_)
	{
		m_space = space_;
	}

	public DummyNode node ()
	{
		return new DummyNode ();
	}

	private DummyBulletSpace m_space;
}
