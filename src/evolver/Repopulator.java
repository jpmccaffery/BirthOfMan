package evolver;

import network.Network;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Collections;
import java.util.Iterator;
import java.util.List;
import java.util.Random;
import network.Connection;
import network.NetworkBuilder;
import network.NetworkXMLWriter;
import network.Neuron;

/**
 *
 * @author Derek
 */
public class Repopulator {
    
    private double chanceToMutateConnectionWeight = 0.08;
    private double chanceToAddConnection = 0.003;
    private double chanceToRemoveConnection = 0.001;
    private double chanceToAddNode = 0.001;
    private double standardDev = 0.05;
    private double mean = 0;
    
    public void repopulate(Integer[] mostFit, Integer[] leastFit) {
        
        for (int i = 0; i < leastFit.length; i++) {
            int newOffspringId = leastFit[i];
            //randomly choose two individuals from most fit to be parents
            Collections.shuffle(Arrays.asList(mostFit));
            int parent1id = mostFit[0];
            int parent2id = mostFit[1];
            
            Network newOffspring = new Network();
            File file = new File("networks/network" + parent1id + ".xml");
            NetworkBuilder networkBuilder = new NetworkBuilder();
            newOffspring = networkBuilder.buildNetworkFromFile( file );
            newOffspring.setId(newOffspringId);
            
            
            //limit number of structural mutations
            int newNeurons = 0;
            int newConns = 0;
            int maxNewNeurons = 3;
            int maxNewConns = 10;
            
            
            //crossover operators
            
            //mutation operators
            Random rand = new Random();
            
            //***Mutate connection weights***
            //iterate across all connections
            //for each, determine if to be mutated
            //if yes, mutate
            Iterator iter = newOffspring.getConnectionList().iterator();
            while ( iter.hasNext() ) {
		Connection connectionToMutate = (Connection) iter.next();
                double oldWeight = connectionToMutate.getWeight();
                //System.out.println("weight unchanged: " + oldWeight);
                if (rand.nextInt(100) < (100*chanceToMutateConnectionWeight)) {
                    double newWeight = oldWeight + ( (standardDev * rand.nextGaussian()) + mean );
                    connectionToMutate.setWeight(newWeight);
                    //System.out.println("old weight: " + oldWeight);
                    //System.out.println("new weight: " + newWeight);
                }
                if (rand.nextInt(100) < (100*chanceToRemoveConnection)) {
//                  System.out.println("Connection " + connectionToMutate.getId() + " removed!");
                    iter.remove();
                }
            }
            

            
            //***Add new connections***
            ArrayList<Neuron> neuronsToCheck = (ArrayList<Neuron>) newOffspring.getNeuronList();
            Iterator iter1 = neuronsToCheck.iterator();
            while (iter1.hasNext()) {
                Neuron n1 = (Neuron) iter1.next();
                long neuron1id = n1.getId();
                Iterator iter2 = neuronsToCheck.iterator();
                while (iter2.hasNext()) {
                    Neuron n2 = (Neuron) iter2.next();
                    long neuron2id = n2.getId();
                    if (neuron1id != neuron2id) {
                        if (!newOffspring.checkIfConnected(neuron1id, neuron2id)) {
                            //System.out.println("neurons not connected: " + neuron1id + "  " + neuron2id);
                            if (rand.nextInt(100) < (100*chanceToAddConnection)) {
                                if (newConns < maxNewConns) {
                                    newOffspring.connectNeurons(neuron1id, neuron2id); 
                                    newConns++;
                                    //System.out.println("New Conn Added to new network " + leastFit[i]);
                                }
                            }
                        }
                    }
                }                
            }
            
            //***Add new neurons***
            newOffspring.clearConnsToAdd();
            Iterator iter2 = newOffspring.getConnectionList().iterator();
            while ( iter2.hasNext() ) {
		Connection connectionToMutate = (Connection) iter2.next();
                if (rand.nextInt(100) < (100*chanceToAddNode)) {
                    if (newNeurons < maxNewNeurons) {
                        newOffspring.mutateNeuron(connectionToMutate.getId());
                        newNeurons++;
                        //System.out.println("New Neuron Added to new network " + leastFit[i]);
                    }
                }
            }
            newOffspring.addBatchConnections();
            
            NetworkXMLWriter xmlWriter = new NetworkXMLWriter();
            String networkXMLString = xmlWriter.writeNetworkToXML(newOffspring);
            
            try {
                File saveFile = new File("networks/network" + newOffspringId + ".xml");
                BufferedWriter out = new BufferedWriter(new FileWriter(saveFile));
                out.write(networkXMLString);
                out.close();
            } catch (IOException exception) {        	    			
            }


        }
        
    }
    
}
