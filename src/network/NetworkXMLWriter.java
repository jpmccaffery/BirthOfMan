/*
 * Copyright (C) 2007 Derek James 
 * 
 * This file is part of SIPHON (Simulating the Phylogeny and Ontogeny of the Neocortex).
 * 
 * SIPHON is free software; you can redistribute it and/or modify it under the terms of the GNU
 * General Public License as published by the Free Software Foundation; either version 2 of the
 * License, or (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful, but WITHOUT ANY WARRANTY;
 * without even the implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See
 * the GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License along with this program; if
 * not, write to the Free Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307 USA
 * 
 * created by Derek James on November 4th, 2007
 */
package network;

import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Iterator;

public class NetworkXMLWriter {

/* This class generates and returns a network XML file from a network object. 
 * All units listed in xml format are assumed to be micrometers
 * unless otherwise specified.
*/

public String writeNetworkToXML (Network networkToWrite){
	
	StringBuffer networkStringBuffer = new StringBuffer();

	networkStringBuffer.append( "<network" );
	networkStringBuffer.append( " " ).append( "id" ).append( "=\"" ).append(
			networkToWrite.getId() ).append( "\"" );
	networkStringBuffer.append( " " ).append( "timeConstant" ).append( "=\"" ).append(
			networkToWrite.getTimeConstant() ).append( "\"" );
	networkStringBuffer.append( " " ).append( "deltaTime" ).append( "=\"" ).append(
			networkToWrite.getDeltaTime() ).append( "\"" );
	networkStringBuffer.append( " " ).append( "type" ).append( "=\"" ).append(
			networkToWrite.getType() );
	networkStringBuffer.append( "\">\n" );

	Iterator iter = networkToWrite.getNeuronList().iterator();
	while ( iter.hasNext() ) {
		Neuron neuronToWrite = (Neuron) iter.next();
		networkStringBuffer.append( "<neuron" );
		networkStringBuffer.append( " " ).append( "id" ).append( "=\"" ).append(
				neuronToWrite.getId() ).append( "\"" );
		networkStringBuffer.append( " " ).append( "excitatory" ).append( "=\"" ).append(
				neuronToWrite.getExcitatory() ).append( "\"" );
		networkStringBuffer.append( " " ).append( "type" ).append( "=\"" ).append(
				neuronToWrite.getType() ).append( "\"" );
		if (neuronToWrite.getType().toString().equals("delay")) {
			networkStringBuffer.append( " " ).append( "parentId" ).append( "=\"" ).append(
					neuronToWrite.getParentId() ).append( "\"" );
			networkStringBuffer.append( " " ).append( "delay" ).append( "=\"" ).append(
					neuronToWrite.getDelay() ).append( "\"" );
		}
		float coords[] = neuronToWrite.getLocation();
		networkStringBuffer.append( " " ).append( "x" ).append( "=\"" ).append(
				coords[0] ).append( "\"" );
		networkStringBuffer.append( " " ).append( "y" ).append( "=\"" ).append(
				coords[1] ).append( "\"" );
		networkStringBuffer.append( " " ).append( "z" ).append( "=\"" ).append(
				coords[2] );
		networkStringBuffer.append( "\" />\n" );
	}

	iter = networkToWrite.getConnectionList().iterator();
	while ( iter.hasNext() ) {
		Connection connectionToWrite = (Connection) iter.next();
		networkStringBuffer.append( "<connection" );
		networkStringBuffer.append( " " ).append( "id" ).append( "=\"" ).append(
				connectionToWrite.getId() ).append( "\"" );
		networkStringBuffer.append( " " ).append( "preSynapticId" ).append( "=\"" ).append(
				connectionToWrite.getPresynapticId() ).append( "\"" );
		networkStringBuffer.append( " " ).append( "postSynapticId" ).append( "=\"" ).append(
				connectionToWrite.getPostsynapticId() ).append( "\"" );
		float coords[] = connectionToWrite.getSrcAndDestLocation();
		networkStringBuffer.append( " " ).append( "srcX" ).append( "=\"" ).append(
				coords[0] ).append( "\"" );
		networkStringBuffer.append( " " ).append( "srcY" ).append( "=\"" ).append(
				coords[1] ).append( "\"" );
		networkStringBuffer.append( " " ).append( "srcZ" ).append( "=\"" ).append(
				coords[2] ).append( "\"" );
		networkStringBuffer.append( " " ).append( "destX" ).append( "=\"" ).append(
				coords[3] ).append( "\"" );
		networkStringBuffer.append( " " ).append( "destY" ).append( "=\"" ).append(
				coords[4] ).append( "\"" );
		networkStringBuffer.append( " " ).append( "destZ" ).append( "=\"" ).append(
				coords[5] ).append( "\"" );
		networkStringBuffer.append( " " ).append( "weight" ).append( "=\"" ).append(
				connectionToWrite.getWeight() ).append( "\"" );
		//networkStringBuffer.append( " " ).append( "delay" ).append( "=\"" ).append(
		//		connectionToWrite.getDelay() ).append( "\"" );
		networkStringBuffer.append( " />\n" );
	}

	
	networkStringBuffer.append( "</" ).append( "network" ).append( ">" );


	return networkStringBuffer.toString();
}

	
}
