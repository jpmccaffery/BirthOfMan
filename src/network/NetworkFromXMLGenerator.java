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
import java.util.ArrayList;
import org.w3c.dom.*;

import javax.xml.parsers.DocumentBuilderFactory;
import javax.xml.parsers.DocumentBuilder;
import org.xml.sax.SAXException;
import org.xml.sax.SAXParseException; 

public class NetworkFromXMLGenerator {

/* This class generates a network XML file readable by SIPHON from an XML file
 * specifying groups of neurons and the connectivity between them.
*/
    
private static File inputXMLFile;
private static long elementID = 1;
private static long[] parentIDs;
private static StringBuffer networkStringBuffer = new StringBuffer();
private static float x = 0.0f;
private static float y = 0.0f;
private static float z = 0.0f;
private static ArrayList groups = new ArrayList();
private static ArrayList neurons;
private static Neuron newNeuron;
    
public static void main(String[] args) {
    inputXMLFile = new File(args[0]); 
    readParamsFromFile();
}
	
public static void readParamsFromFile (){
	
try {
    DocumentBuilderFactory docBuilderFactory = DocumentBuilderFactory.newInstance();
    DocumentBuilder docBuilder = docBuilderFactory.newDocumentBuilder();
    Document doc = docBuilder.parse( inputXMLFile );

    Node rootNode = doc.getDocumentElement();
    Node networkIdNode = rootNode.getAttributes().getNamedItem("id");
    Node timeConstantNode = rootNode.getAttributes().getNamedItem("timeConstant");
    Node deltaTimeNode = rootNode.getAttributes().getNamedItem("deltaTime");
    Node typeNode = rootNode.getAttributes().getNamedItem("type");
    long newNetworkId = Long.parseLong(networkIdNode.getNodeValue());
    double newTimeConstant = Double.parseDouble(timeConstantNode.getNodeValue());
    double newDeltaTime = Double.parseDouble(deltaTimeNode.getNodeValue());
    String newType = typeNode.getNodeValue();
    
    networkStringBuffer.append( "<network" );
    networkStringBuffer.append( " " ).append( "id" ).append( "=\"" ).append(
                    newNetworkId ).append( "\"" );
    networkStringBuffer.append( " " ).append( "timeConstant" ).append( "=\"" ).append(
                    newTimeConstant ).append( "\"" );
    networkStringBuffer.append( " " ).append( "deltaTime" ).append( "=\"" ).append(
                    newDeltaTime ).append( "\"" );
    networkStringBuffer.append( " " ).append( "type" ).append( "=\"" ).append(
                    newType );
    networkStringBuffer.append( "\">\n" );
    
    
    NodeList groupList = doc.getElementsByTagName("group");
      
    for( int i=0; i < groupList.getLength(); i++ ) {
        
    	Node groupNode = groupList.item( i );
    	Node groupNodeId = groupNode.getAttributes().getNamedItem("id");
    	Node groupNodeLayer = groupNode.getAttributes().getNamedItem("layer");
    	Node groupNodeNumNeurons = groupNode.getAttributes().getNamedItem("neurons");
    	Node groupNodeType = groupNode.getAttributes().getNamedItem("type");
        Node groupNodeDelay = groupNode.getAttributes().getNamedItem("delay");
        
        String newGroupId = groupNodeId.getNodeValue();
    	int newGroupLayer = Integer.parseInt(groupNodeLayer.getNodeValue());
    	int newGroupNumNeurons = Integer.parseInt(groupNodeNumNeurons.getNodeValue());
    	String newGroupType = groupNodeType.getNodeValue();
        long newGroupDelay = Long.parseLong(groupNodeDelay.getNodeValue());

        if ( !newGroupType.contains("delay") ) {
            parentIDs = new long[newGroupNumNeurons]; 
            x = 0.0f;
        } 
        if ( !newGroupType.contains("delay") && newGroupLayer != 1 ) {
            y += 100.0;
        } 
        if (newGroupType.contains("delay")) {
            y += 0.1;
        }
        
        neurons = new ArrayList();
        groups.add(neurons);
        
        for (int j=0; j < newGroupNumNeurons; j++ ) {
            networkStringBuffer.append( "<neuron" );
            networkStringBuffer.append( " " ).append( "id" ).append( "=\"" ).append(
				elementID ).append( "\"" );
            if ( newGroupType.contains("input") || newGroupType.contains("hidden") ) {
                  parentIDs[j] = elementID;  
            }
		networkStringBuffer.append( " " ).append( "excitatory" ).append( "=\"" ).append(
				"true" ).append( "\"" );
		networkStringBuffer.append( " " ).append( "type" ).append( "=\"" ).append(
				newGroupType ).append( "\"" );
		if (newGroupType.equals("delay")) {
			networkStringBuffer.append( " " ).append( "parentId" ).append( "=\"" ).append(
					parentIDs[j] ).append( "\"" );
			networkStringBuffer.append( " " ).append( "delay" ).append( "=\"" ).append(
					newGroupDelay ).append( "\"" );
		}
		networkStringBuffer.append( " " ).append( "x" ).append( "=\"" ).append(
				x ).append( "\"" );
		networkStringBuffer.append( " " ).append( "y" ).append( "=\"" ).append(
				y ).append( "\"" );
		networkStringBuffer.append( " " ).append( "z" ).append( "=\"" ).append(
				z );
		networkStringBuffer.append( "\" />\n" );
                
                newNeuron = new LeakyIntegrateAndFireNeuron();
                newNeuron.setLocation(x, y, z);
                newNeuron.setId(elementID);
                neurons.add(newNeuron);
                        
                x += 50.0;
                elementID++;
        }
        
        y = Math.round(y);


    }

  
    NodeList connectionList = doc.getElementsByTagName("connections");
    
    for ( int j=0; j < connectionList.getLength(); j++ ) {
        Node connectionNode = connectionList.item( j );
        Node connectionNodeFrom = connectionNode.getAttributes().getNamedItem("from");
        Node connectionNodeTo = connectionNode.getAttributes().getNamedItem("to");

        int connectionFrom = Integer.parseInt(connectionNodeFrom.getNodeValue());
        int connectionTo = Integer.parseInt(connectionNodeTo.getNodeValue());
        
        ArrayList fromGroup = (ArrayList) groups.get(connectionFrom - 1);
        ArrayList toGroup = (ArrayList) groups.get(connectionTo - 1);
     
        for ( int m = 0; m < fromGroup.size(); m++ ) {
            Neuron preSynNeuron = (Neuron) fromGroup.get(m);
            for ( int n = 0; n < toGroup.size(); n++ ) {
                Neuron postSynNeuron = (Neuron) toGroup.get(n);
                networkStringBuffer.append( "<connection" );
		networkStringBuffer.append( " " ).append( "id" ).append( "=\"" ).append(
				elementID ).append( "\"" );
		networkStringBuffer.append( " " ).append( "excitatory" ).append( "=\"" ).append(
				true ).append( "\"" );
		networkStringBuffer.append( " " ).append( "preSynapticId" ).append( "=\"" ).append(
				preSynNeuron.getId() ).append( "\"" );
		networkStringBuffer.append( " " ).append( "postSynapticId" ).append( "=\"" ).append(
				postSynNeuron.getId() ).append( "\"" );
		float preCoords[] = preSynNeuron.getLocation();
                float postCoords[] = postSynNeuron.getLocation();
		networkStringBuffer.append( " " ).append( "srcX" ).append( "=\"" ).append(
				preCoords[0] ).append( "\"" );
		networkStringBuffer.append( " " ).append( "srcY" ).append( "=\"" ).append(
				preCoords[1] ).append( "\"" );
		networkStringBuffer.append( " " ).append( "srcZ" ).append( "=\"" ).append(
				preCoords[2] ).append( "\"" );
		networkStringBuffer.append( " " ).append( "destX" ).append( "=\"" ).append(
				postCoords[0] ).append( "\"" );
		networkStringBuffer.append( " " ).append( "destY" ).append( "=\"" ).append(
				postCoords[1] ).append( "\"" );
		networkStringBuffer.append( " " ).append( "destZ" ).append( "=\"" ).append(
				postCoords[2] ).append( "\"" );
		networkStringBuffer.append( " " ).append( "weight" ).append( "=\"" ).append(
				"0" ).append( "\"" );
		networkStringBuffer.append( " />\n" );
                elementID++;
            }
        }
        
    }

    networkStringBuffer.append( "</network>" );
    
    }catch (SAXParseException err) {
    System.out.println ("** Parsing error" + ", line " 
         + err.getLineNumber () + ", uri " + err.getSystemId ());
    System.out.println(" " + err.getMessage ());

    }catch (SAXException e) {
    Exception ex = e.getException ();
    ((ex == null) ? e : ex).printStackTrace ();

    }catch (Throwable t) {
    t.printStackTrace ();
    }
    //System.exit (0);
    
    File saveFile = new File("network3000.xml");
    try {
        BufferedWriter out = new BufferedWriter(new FileWriter(saveFile));
        out.write(networkStringBuffer.toString());
        out.close();
    } 
        catch (IOException exception) {        	    			
    }

}


	
}
