package physics.jmonkey;

import physics.AbstractLimb;

import utilities.CoordUtils;

import com.jme3.app.LegacyApplication;

import com.jme3.asset.DesktopAssetManager;

import com.jme3.bullet.PhysicsSpace;
import com.jme3.bullet.collision.shapes.CapsuleCollisionShape;
import com.jme3.bullet.control.RigidBodyControl;

import com.jme3.material.Material;

import com.jme3.math.ColorRGBA;
import com.jme3.math.Quaternion;
import com.jme3.math.Vector3f;

import com.jme3.renderer.queue.RenderQueue.ShadowMode;

import com.jme3.scene.shape.Cylinder;
import com.jme3.scene.Geometry;
import com.jme3.scene.Node;


public class MonkeyLimb implements AbstractLimb, MonkeyObject
{
	public MonkeyLimb (AbstractLimb limb_)
	{
		m_node = new Node ("Limb");
		m_numCaps = limb_.numCaps ();

		int axis = PhysicsSpace.AXIS_Z;

		CapsuleCollisionShape shape =
			new CapsuleCollisionShape (RADIUS, limb_.length (), axis);
		RigidBodyControl rigidBodyControl =
			new RigidBodyControl (shape, limb_.mass ());

		rigidBodyControl.setFriction(FRICTION);
		rigidBodyControl.setLinearVelocity(limb_.speed ());

		m_node.setLocalRotation (limb_.orientation ());
		m_node.setLocalTranslation (limb_.position ());
		m_node.addControl (rigidBodyControl);
		m_node.setShadowMode (ShadowMode.Cast);

		float capLen = CAP_SIZE * limb_.length ();
		float bodyLen = (1f - m_numCaps * CAP_SIZE) * limb_.length ();
		float offset = -limb_.length () / 2f + bodyLen / 2f;

		if (m_numCaps > 1)
			offset += capLen;

		Geometry cylinderGeo;

		cylinderGeo = makeCylinder (bodyLen, new Vector3f (0f, 0f, offset), ColorRGBA.Red);
		m_node.attachChild(cylinderGeo);

		if (m_numCaps < 1)
			return;

		offset = limb_.length () / 2f - capLen / 2f;
		cylinderGeo = makeCylinder (capLen, new Vector3f (0f, 0f, offset), ColorRGBA.Blue);
		m_node.attachChild(cylinderGeo);

		if (m_numCaps < 2)
			return;

		offset = -limb_.length () / 2f + capLen / 2f;
		cylinderGeo = makeCylinder (capLen, new Vector3f (0f, 0f, offset), ColorRGBA.Blue);
		m_node.attachChild(cylinderGeo);
	}

	private Geometry makeCylinder (float length_, Vector3f center_, ColorRGBA color_)
	{
		Cylinder limbBox = new Cylinder (AXIS_SAMPLES, RADIAL_SAMPLES,
		                                 RADIUS, length_, true);
		Geometry limbGeometry = new Geometry (LIMB_NAME, limbBox);

		DesktopAssetManager manager = new DesktopAssetManager (true);
		Material material = new Material(manager, "Common/MatDefs/Light/Lighting.j3md");

		material.setBoolean ("UseMaterialColors",true);
		material.setColor ("Ambient", color_);
		material.setColor ("Diffuse", color_);
		material.setColor ("Specular", ColorRGBA.White);
		material.setFloat ("Shininess", 12);

		limbGeometry.setMaterial (material);
		limbGeometry.setLocalTranslation (center_);

		return limbGeometry;
	}

	public Vector3f speed ()
	{
		return m_node.getControl (RigidBodyControl.class).getLinearVelocity ();
	}

	public void setSpeed (Vector3f speed_)
	{
		m_node.getControl (RigidBodyControl.class).setLinearVelocity (speed_);
	}

	public float length ()
	{
		return ((Cylinder) ((Geometry) m_node.getChild (LIMB_NAME)).getMesh ()).getHeight ();
	}

	public void setLength (float length_)
	{
		System.err.println ("MonkeyLimb::setLength: Invalid update. Reparent first.");
		System.exit (0);
	}

	public float mass ()
	{
		return m_node.getControl (RigidBodyControl.class).getMass ();
	}

	public void setMass (float mass_)
	{
		m_node.getControl (RigidBodyControl.class).setMass (mass_);
	}

	public Quaternion orientation ()
	{
		return m_node.getLocalRotation ();
	}

	public void setOrientation (Quaternion orientation_)
	{
		m_node.setLocalRotation (orientation_);
	}

	public Vector3f alignment ()
	{
		return CoordUtils.zAxisFromRotation (orientation ());
	}

	public void setAlignment (Vector3f alignment_)
	{
		setOrientation (CoordUtils.rotationFromZAxis (alignment_));
	}

	public int numCaps ()
	{
		return m_numCaps;
	}

	public Vector3f position ()
	{
		return m_node.getLocalTranslation ();
	}

	public void setPosition (Vector3f position_)
	{
		m_node.setLocalTranslation (position_);
	}

	public void unregisterFromSpace ()
	{
		m_space.removeAll (m_node);

		m_node.removeFromParent ();
	}

	/// May need to attach to a root node
	public void registerWithJMonkey (PhysicsSpace space_, Node rootNode_)
	{
		m_space = space_;
		m_space.addAll (m_node);

		rootNode_.attachChild (m_node);
	}

	public Node node ()
	{
		return m_node;
	}

	private Node m_node;
	private PhysicsSpace m_space;
	private int m_numCaps;

	private static final float RADIUS = 0.2f;
	private static final float FRICTION = 0.05f;

	private static final float CAP_SIZE = 0.1f;
	private static final int AXIS_SAMPLES = 32;
	private static final int RADIAL_SAMPLES = 32;
	private static final String LIMB_NAME = "limb";
}
