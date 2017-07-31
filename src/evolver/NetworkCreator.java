/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package evolver;

import java.util.Random;

/**
 *
 * @author Derek
 */
public class NetworkCreator {

        private StringBuffer networkStringBuffer;
        private int runningNeuronId = 0;
        private int runningConnectionId = 0;
    
    public String createNetwork(int id, int numInputs, int numOutputs) {
        
        networkStringBuffer = new StringBuffer();
        runningNeuronId = 0;
        runningConnectionId = 0;
		networkStringBuffer.append( "<network" );
		networkStringBuffer.append( " " ).append( "id" ).append( "=\"" ).append( id ).append( "\"" );
		networkStringBuffer.append( " " ).append( "timeConstant" ).append( "=\"" ).append( "10.0" ).append( "\"" );
		networkStringBuffer.append( " " ).append( "deltaTime" ).append( "=\"" ).append( "0.02" ).append( "\"" );
		networkStringBuffer.append( " " ).append( "type" ).append( "=\"" ).append( "leakyIntegrator" );
		networkStringBuffer.append( "\">\n" );

        for (int i = 0; i < numInputs; i++) {
		networkStringBuffer.append( "<neuron" );
		networkStringBuffer.append( " " ).append( "id" ).append( "=\"" ).append( runningNeuronId ).append( "\"" );
		networkStringBuffer.append( " " ).append( "type" ).append( "=\"" ).append("input" ).append( "\"" );
                /*
		if (neuronToWrite.getType().toString().equals("delay")) {
			networkStringBuffer.append( " " ).append( "parentId" ).append( "=\"" ).append(
					neuronToWrite.getParentId() ).append( "\"" );
			networkStringBuffer.append( " " ).append( "delay" ).append( "=\"" ).append(
					neuronToWrite.getDelay() ).append( "\"" );
		}
                 * 
                 */
		float coords[] = new float[3];
                coords[0] = 0;
                coords[1] = i*100;
                coords[2] = 0;
		networkStringBuffer.append( " " ).append( "x" ).append( "=\"" ).append(
				coords[0] ).append( "\"" );
		networkStringBuffer.append( " " ).append( "y" ).append( "=\"" ).append(
				coords[1] ).append( "\"" );
		networkStringBuffer.append( " " ).append( "z" ).append( "=\"" ).append(
				coords[2] );
		networkStringBuffer.append( "\" />\n" );
                runningNeuronId++;
	}

        for (int j = 0; j < numOutputs; j++) {
		networkStringBuffer.append( "<neuron" );
		networkStringBuffer.append( " " ).append( "id" ).append( "=\"" ).append( runningNeuronId ).append( "\"" );
		networkStringBuffer.append( " " ).append( "type" ).append( "=\"" ).append("output" ).append( "\"" );
		float coords[] = new float[3];
                coords[0] = 500000;
                coords[1] = j*100;
                coords[2] = 0;
		networkStringBuffer.append( " " ).append( "x" ).append( "=\"" ).append(
				coords[0] ).append( "\"" );
		networkStringBuffer.append( " " ).append( "y" ).append( "=\"" ).append(
				coords[1] ).append( "\"" );
		networkStringBuffer.append( " " ).append( "z" ).append( "=\"" ).append(
				coords[2] );
		networkStringBuffer.append( "\" />\n" );
                runningNeuronId++;
	}

        runningConnectionId = runningNeuronId + 1;
        
        
        for (int i = 0; i < numInputs; i++) {
            for (int j = numInputs; j < numInputs+numOutputs; j++) {
		networkStringBuffer.append( "<connection" );
		networkStringBuffer.append( " " ).append( "id" ).append( "=\"" ).append( runningConnectionId ).append( "\"" );
		networkStringBuffer.append( " " ).append( "preSynapticId" ).append( "=\"" ).append( i ).append( "\"" );
		networkStringBuffer.append( " " ).append( "postSynapticId" ).append( "=\"" ).append( j ).append( "\"" );
		float coords[] = new float[6];
                coords[0] = 0;
                coords[1] = i*100;
                coords[2] = 0;
                coords[3] = 500000;
                coords[4] = (j-numInputs)*100;
                coords[5] = 0;
                
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
                
                Random rand = new Random();
                double standardDev = 0.30;
                double mean = 0;
		double newWeight = ( (standardDev * rand.nextGaussian()) + mean );
	
		networkStringBuffer.append( " " ).append( "weight" ).append( "=\"" ).append( newWeight ).append( "\"" );
		//networkStringBuffer.append( " " ).append( "delay" ).append( "=\"" ).append(
		//		connectionToWrite.getDelay() ).append( "\"" );
		networkStringBuffer.append( " />\n" );
                runningConnectionId++;
            }
	}        
        
	
	networkStringBuffer.append( "</" ).append( "network" ).append( ">" );


	return networkStringBuffer.toString();
        
    }
    
    
}
