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

import java.io.File;
import org.w3c.dom.Document;
import org.w3c.dom.*;

import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.DocumentBuilder;
import org.xml.sax.SAXException;
import org.xml.sax.SAXParseException; 

public class NetworkBuilder {

/* This class builds a working network from a network.xml file 
 * All units listed in xml format are assumed to be micrometers
 * unless otherwise specified.
*/
	
public Network buildNetworkFromFile (File networkXMLFile){
	
Network newNetwork = new Network();

try {
    DocumentBuilderFactory docBuilderFactory = DocumentBuilderFactory.newInstance();
    DocumentBuilder docBuilder = docBuilderFactory.newDocumentBuilder();
    Document doc = docBuilder.parse( networkXMLFile );

    Node rootNode = doc.getDocumentElement();
    Node networkIdNode = rootNode.getAttributes().getNamedItem("id");
    Node timeConstantNode = rootNode.getAttributes().getNamedItem("timeConstant");
    Node deltaTimeNode = rootNode.getAttributes().getNamedItem("deltaTime");
    Node typeNode = rootNode.getAttributes().getNamedItem("type");
    long newNetworkId = Long.parseLong(networkIdNode.getNodeValue());
    double newTimeConstant = Double.parseDouble(timeConstantNode.getNodeValue());
    double newDeltaTime = Double.parseDouble(deltaTimeNode.getNodeValue());
    String newType = typeNode.getNodeValue();
    newNetwork.setId(newNetworkId);
    newNetwork.setTimeConstant(newTimeConstant);
    newNetwork.setDeltaTime(newDeltaTime);
    newNetwork.setType(newType);
    
    NodeList neuronList = doc.getElementsByTagName("neuron");
    int totalNeurons = neuronList.getLength();
    //System.out.println("Total no of neurons : " + totalNeurons);

    NodeList connectionList = doc.getElementsByTagName("connection");
    int totalConnections = connectionList.getLength();
    //System.out.println("Total no of connections : " + totalConnections);
    
    for( int i=0; i < neuronList.getLength(); i++ ){
    	Node neuronNode = neuronList.item( i );
    	Node neuronNodeId = neuronNode.getAttributes().getNamedItem("id");
    	Node neuronNodeType = neuronNode.getAttributes().getNamedItem("type");
    	Node neuronNodeX = neuronNode.getAttributes().getNamedItem("x");
    	Node neuronNodeY = neuronNode.getAttributes().getNamedItem("y");
    	Node neuronNodeZ = neuronNode.getAttributes().getNamedItem("z");
    	String newNeuronType = neuronNodeType.getNodeValue();
    	long newNeuronId = Long.parseLong(neuronNodeId.getNodeValue());
    	float newNeuronX = Float.parseFloat(neuronNodeX.getNodeValue());
    	float newNeuronY = Float.parseFloat(neuronNodeY.getNodeValue());
    	float newNeuronZ = Float.parseFloat(neuronNodeZ.getNodeValue());
    	Neuron newNeuron = null;
        if (typeNode.getNodeValue().toString().equals("leakyIntegrateAndFire")) {
        	newNeuron = new LeakyIntegrateAndFireNeuron();
        	newNeuron.setType(newNeuronType);
        	newNeuron.setId(newNeuronId);
        	newNeuron.setLocation(newNeuronX, newNeuronY, newNeuronZ); 
                newNeuron.setDeltaT(newDeltaTime);
                newNeuron.setTimeConstant(newTimeConstant);
        } 
        if (typeNode.getNodeValue().toString().equals("leakyIntegrator")) {
        	newNeuron = new LeakyIntegratorNeuron();
        	newNeuron.setType(newNeuronType);
        	newNeuron.setId(newNeuronId);
        	newNeuron.setLocation(newNeuronX, newNeuronY, newNeuronZ); 
                newNeuron.setDeltaT(newDeltaTime);
                newNeuron.setTimeConstant(newTimeConstant);
        }
        if ( newNeuron.getType().toString().equals("delay")) {
        	//set delay and parentID
        	Node neuronNodeDelay = neuronNode.getAttributes().getNamedItem("delay");
        	Node neuronNodeParentId = neuronNode.getAttributes().getNamedItem("parentId");
        	int newNeuronDelay = Integer.parseInt(neuronNodeDelay.getNodeValue());
        	long newNeuronParentId = Long.parseLong(neuronNodeParentId.getNodeValue());
        	newNeuron.setDelay(newNeuronDelay);
        	newNeuron.setParentId(newNeuronParentId);
                newNeuron.setDeltaT(newDeltaTime);
                newNeuron.setTimeConstant(newTimeConstant);
        }
        newNetwork.addNeuron( newNeuron );
        
    	//Element neuronElement = (Element)neuronNode;

    	//NodeList childConnections = neuronElement.getElementsByTagName("connection");



    }//end of for loop 

    for ( int j=0; j < connectionList.getLength(); j++ ) {
		Node connectionNode = connectionList.item( j );
		Node connectionNodeId = connectionNode.getAttributes().getNamedItem("id");
		Node connectionNodeWeight = connectionNode.getAttributes().getNamedItem("weight");
		Node connectionNodepreSynapticId = connectionNode.getAttributes().getNamedItem("preSynapticId");
		Node connectionNodepostSynapticId = connectionNode.getAttributes().getNamedItem("postSynapticId");
		Node connectionNodeSrcX = connectionNode.getAttributes().getNamedItem("srcX");
		Node connectionNodeSrcY = connectionNode.getAttributes().getNamedItem("srcY");
		Node connectionNodeSrcZ = connectionNode.getAttributes().getNamedItem("srcZ");
		Node connectionNodeDestX = connectionNode.getAttributes().getNamedItem("destX");
		Node connectionNodeDestY = connectionNode.getAttributes().getNamedItem("destY");
		Node connectionNodeDestZ = connectionNode.getAttributes().getNamedItem("destZ");
		//Node connectionNodeDelay = connectionNode.getAttributes().getNamedItem("delay");
		long newConnectionId = Long.parseLong(connectionNodeId.getNodeValue());
		double newWeight = Double.parseDouble(connectionNodeWeight.getNodeValue());
		long newPresynapticId = Long.parseLong(connectionNodepreSynapticId.getNodeValue());
		long newPostsynapticId = Long.parseLong(connectionNodepostSynapticId.getNodeValue());
		float newSrcX = Float.parseFloat(connectionNodeSrcX.getNodeValue());
		float newSrcY = Float.parseFloat(connectionNodeSrcY.getNodeValue());
		float newSrcZ = Float.parseFloat(connectionNodeSrcZ.getNodeValue());
		float newDestX = Float.parseFloat(connectionNodeDestX.getNodeValue());
		float newDestY = Float.parseFloat(connectionNodeDestY.getNodeValue());
		float newDestZ = Float.parseFloat(connectionNodeDestZ.getNodeValue());
		//int newDelay = Integer.parseInt(connectionNodeDelay.getNodeValue());
		//add new connection with attributes gathered from XML
		Connection newConnection = new Connection();
		newConnection.setId(newConnectionId);
		newConnection.setWeight(newWeight);
		newConnection.setPresynapticId(newPresynapticId);
		newConnection.setPostsynapticId(newPostsynapticId);
		newConnection.setSrcAndDestLocation(newSrcX, newSrcY, newSrcZ, newDestX, newDestY, newDestZ);
		//newConnection.setDelay(newDelay);
		newNetwork.addConnection(newConnection);
	}//end of inner for loop

    }catch (SAXParseException err) {
    System.out.println ("** Parsing error" + ", line " 
         + err.getLineNumber () + ", uri " + err.getSystemId ());
    System.out.println(" " + err.getMessage ());

    }catch (SAXException e) {
    Exception x = e.getException ();
    ((x == null) ? e : x).printStackTrace ();

    }catch (Throwable t) {
    t.printStackTrace ();
    }
    //System.exit (0);
    
    newNetwork.updateAllConnectivity();
    return newNetwork;
}


	
}
