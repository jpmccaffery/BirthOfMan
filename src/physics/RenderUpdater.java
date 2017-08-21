package physics;

import com.jme3.scene.Node;


public interface RenderUpdater
{
	public void registerUpdatable (RenderUpdatable u_);
	public void unregisterUpdatable (RenderUpdatable u_);

	public Node rootNode ();
}
