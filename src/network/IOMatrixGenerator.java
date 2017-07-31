package network;


import java.util.Collection;
import java.util.Iterator;


/* Assigns neurons in the network as either input or output units and arranges
 * them in a matrix based on their topography. The default algorithm for defining
 * the input and output matrices is to first find the minimum and maximum values
 * of y for all neurons. The input matrix is composed of all neurons with the min
 * value, and the output matrix is composed of all neurons with the max value.
 *  
 * TODO: Extend this class with additional algorithms for defining IO matrices.
 */

public class IOMatrixGenerator {

private Neuron[][] inputMatrix;	
private Neuron[][] outputMatrix;
private float minY = 0.0f;
private float maxY = 1.0f;
	
public Neuron[][] generateInputMatrix(Collection neurons) {
	
	Iterator neuronIt = neurons.iterator();
	while ( neuronIt.hasNext() ) {
		Neuron n = (Neuron) neuronIt.next();
		if (n.getLocation()[1] == minY) {
			n.setType("input");
		}
	}
	return inputMatrix;
}

public Neuron[][] generateOutputMatrix(Collection neurons) {
	Iterator neuronIt = neurons.iterator();
	while ( neuronIt.hasNext() ) {
		Neuron n = (Neuron) neuronIt.next();
		if (n.getLocation()[1] == maxY) {
			n.setType("output");
		}
	}
	return outputMatrix;
}

}
