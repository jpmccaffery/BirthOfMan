package mygame;

import evolver.CreaturesMain;

public class Main
{
	public static void main (String[] args_)
	{
		CreaturesMain app = new CreaturesMain ();

		app.setPauseOnLostFocus (false);
		app.start ();
	}
}
