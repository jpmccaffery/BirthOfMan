package physics.jmonkey;

import physics.PhysicsObject;

import com.jme3.bullet.PhysicsSpace;

import com.jme3.scene.Node;


/// \brief Basic JMonkey physics object
public interface MonkeyObject extends PhysicsObject
{
	public void registerWithJMonkey (PhysicsSpace space_, Node rootNode_);
}
