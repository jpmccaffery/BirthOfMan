package utilities;

import com.jme3.math.Vector3f;


public class LineUtils
{
	/// \brief Simple line case
	public static class Line
	{
		public Line (Vector3f offset_, Vector3f direction_)
		{
			m_offset = offset_;
			m_direction = direction_;
		}

		public Vector3f offset ()
		{
			return m_offset;
		}

		public Vector3f direction ()
		{
			return m_direction;
		}

		private final Vector3f m_offset;
		private final Vector3f m_direction;
	}

	/// http://geomalgorithms.com/a07-_distance.html
	///
	/// \brief Find the midpoint of the narrowest spot connecting
	/// the two lines
	public static Vector3f closestPoint (Line lineA_, Line lineB_)
	{
		Vector3f P = lineA_.offset ();
		Vector3f u = lineA_.direction ();
		Vector3f Q = lineB_.offset ();
		Vector3f v = lineB_.direction ();

		float a = u.dot (u);
		float b = u.dot (v);
		float c = v.dot (v);

		Vector3f w = P.subtract (Q);

		float d = u.dot (w);
		float e = v.dot (w);

		float disc = (a * c - b * b);

		// If the lines are parallel just return half between the two points
		if (disc * disc < THRESHOLD)
		{
			System.err.println ("LineUtils::closestPoint: zero divisor");
			return (P.add (Q)).mult (0.5f);
		}

		float s = (b * e - c * d) / disc;
		float t = (a * e - b * d) / disc;

		Vector3f A = P.add (u.mult (s));
		Vector3f B = Q.add (v.mult (t));

		return (A.add (B)).mult (0.5f);
	}

	/// https://en.wikipedia.org/wiki/Distance_from_a_point_to_a_line
	///
	/// \brief Find the distance between a point and a line
	public static float distance (Line line_, Vector3f point_)
	{
		Vector3f n = line_.direction ().normalize ();
		Vector3f a = line_.offset ();
		Vector3f p = point_;

		Vector3f a_p = a.subtract (p);

		return (a_p.subtract (n.mult (a_p.dot (n)))).length ();
	}

	private static final float THRESHOLD = 0.001f;
}
