package critter;

import critter.body.SimpleBody;
import critter.brain.BrainVat;

import java.util.List;
import java.util.ArrayList;


public class SimpleWalkerBirthingPod extends WalkerBirthingPod
{
	public SimpleWalkerBirthingPod (BrainVat vat_)
	{
		super (vat_, new SimpleBody ());
	}
}
