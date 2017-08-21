package physics.jmonkey;

import physics.PhysicsObject;
import physics.RenderUpdatable;
import physics.RenderUpdater;

import com.jme3.bullet.PhysicsSpace;


/// \brief Basic JMonkey physics object
public interface MonkeyObject extends PhysicsObject, RenderUpdatable
{
	public void registerWithJMonkey (PhysicsSpace space_, RenderUpdater updater_);
}
