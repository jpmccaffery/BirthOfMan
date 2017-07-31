package physics;

import physics.aether.AetherLimb;
import physics.aether.AetherObject;

import physics.bullet.BulletLimb;
import physics.bullet.DummyBulletSpace;
import physics.bullet.DummyNode;

import physics.jmonkey.MonkeyLimb;

import com.jme3.bullet.PhysicsSpace;

import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;

import com.jme3.scene.Node;


public class Limb implements AbstractLimb, CompositeObject
{
	public Limb ()
	{
		m_limb = new AetherLimb ();
	}

	public Limb (AbstractLimb limb_)
	{
		m_limb = limb_;
	}

	public Limb (Vector3f position_, Vector3f alignment_, float length_, float mass_,
	             int numCaps_)
	{
		m_limb = new AetherLimb (position_, alignment_, length_, mass_, numCaps_);
	}

	public Limb (Vector3f position_, Vector3f alignment_, float length_, float mass_)
	{
		m_limb = new AetherLimb (position_, alignment_, length_, mass_);
	}

	public Limb (Vector3f position_, float length_, float mass_)
	{
		m_limb = new AetherLimb (position_, length_, mass_);
	}

	public Limb (Vector3f position_, float length_)
	{
		m_limb = new AetherLimb (position_, length_);
	}

	public Limb (float length_)
	{
		m_limb = new AetherLimb (length_);
	}

	public Vector3f speed ()
	{
		return m_limb.speed ();
	}

	public void setSpeed (Vector3f speed_)
	{
		m_limb.setSpeed (speed_);
	}

	public float length ()
	{
		return m_limb.length ();
	}

	public void setLength (float length_)
	{
		m_limb.setLength (length_);
	}

	public float mass ()
	{
		return m_limb.mass ();
	}

	public void setMass (float mass_)
	{
		m_limb.setMass (mass_);
	}

	public Quaternion orientation ()
	{
		return m_limb.orientation ();
	}

	public void setOrientation (Quaternion orientation_)
	{
		m_limb.setOrientation (orientation_);
	}

	public Vector3f alignment ()
	{
		return m_limb.alignment ();
	}

	public void setAlignment (Vector3f alignment_)
	{
		m_limb.setAlignment (alignment_);
	}

	public void unregisterFromSpace ()
	{
		m_limb.unregisterFromSpace ();
	}

	public int numCaps ()
	{
		return m_limb.numCaps ();
	}

	public Vector3f position ()
	{
		return m_limb.position ();
	}

	public void setPosition (Vector3f position_)
	{
		m_limb.setPosition (position_);
	}

	public void registerWithBullet (DummyBulletSpace space_)
	{
		m_limb.unregisterFromSpace ();
		BulletLimb bLimb = new BulletLimb (m_limb);
		bLimb.registerWithBullet (space_);

		m_limb = bLimb;
	}

	public void registerWithJMonkey (PhysicsSpace space_, Node rootNode_)
	{
		m_limb.unregisterFromSpace ();
		MonkeyLimb mLimb = new MonkeyLimb (m_limb);
		mLimb.registerWithJMonkey (space_, rootNode_);

		m_limb = mLimb;
	}

	// This is a horrible hack
	public Node monkeyNode ()
	{
		if (! (m_limb instanceof MonkeyLimb))
		{
			System.out.println ("Attempt to access monkey node");
			System.exit (0);
		}

		return ((MonkeyLimb) m_limb).node ();
	}

	// This is a horrible hack
	public DummyNode bulletNode ()
	{
		if (! (m_limb instanceof BulletLimb))
		{
			System.out.println ("Attempt to access bullet node");
			System.exit (0);
		}

		return ((BulletLimb) m_limb).node ();
	}

	private AbstractLimb m_limb;
}
