package critter.body;

import physics.CompositeObject;
import physics.Joint;
import physics.Limb;
import physics.bullet.DummyBulletSpace;

import com.jme3.bullet.PhysicsSpace;
import com.jme3.math.Vector3f;
import com.jme3.scene.Node;

import java.util.List;


public abstract class Body implements CompositeObject
{
	public abstract List<Limb> limbs ();
	public abstract List<Joint> joints ();

	public Vector3f position ()
	{
		Vector3f average = new Vector3f (0f, 0f, 0f);

		for (Limb l : limbs ())
			average.add (l.position ());

		return average.mult (1f / limbs ().size ());
	}

	public final void unregisterFromSpace ()
	{
		for (Joint j : joints ())
			j.unregisterFromSpace ();

		for (Limb l : limbs ())
			l.unregisterFromSpace ();
	}

	public final void registerWithJMonkey (PhysicsSpace space_, Node rootNode_)
	{
		for (Limb l : limbs ())
			l.registerWithJMonkey (space_, rootNode_);

		for (Joint j : joints ())
			j.registerWithJMonkey (space_, rootNode_);
	}

	public final void registerWithBullet (DummyBulletSpace space_)
	{
		for (Limb l : limbs ())
			l.registerWithBullet (space_);

		for (Joint j : joints ())
			j.registerWithBullet (space_);
	}
}
