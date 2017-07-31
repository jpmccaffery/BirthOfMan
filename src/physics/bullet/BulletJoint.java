package physics.bullet;

import physics.AbstractJoint;
import physics.Limb;

import com.jme3.math.Vector3f;


public class BulletJoint implements AbstractJoint, BulletObject
{
	public BulletJoint (AbstractJoint joint_)
	{
		System.out.println ("Bullet physics not supported.");
		System.exit (0);
	}

	public Vector3f angularSpeed ()
	{
		return new Vector3f (0, 0, 0);
	}

	public void setAngularSpeed (Vector3f speed_)
	{
	}

	public Limb leftJoin ()
	{
		return new Limb ();
	}

	public Limb rightJoin ()
	{
		return new Limb ();
	}

	public void unregisterFromSpace ()
	{
	}

	public Vector3f position ()
	{
		return new Vector3f (0, 0, 0);
	}

	public void registerWithBullet (DummyBulletSpace space_)
	{
	}
}
