package physics;

public interface Highlightable
{
	default public void highlight ()
	{
	}

	default public void unHighlight ()
	{
	}

	default public boolean isHighlighted ()
	{
		return false;
	}
}
