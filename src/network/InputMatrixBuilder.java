package network;

import java.io.BufferedInputStream;
import java.io.BufferedReader;
import java.io.File;
import java.io.FileInputStream;
import java.io.FileReader;
import java.io.IOException;
import java.io.InputStreamReader;
import java.util.StringTokenizer;

//Takes a file as input and returns a matrix of values to be input into
//the network.

public class InputMatrixBuilder {

double[][] inputMatrix;

public double[][] buildInputMatrixFromFile( File fileToProcess ) {
	
    String lineToRead = null;
    String tokenToRead = null;
    int lineNum = 0;
    int tokenNum = 0;

    //Determine size of matrix
    try { 
    	FileReader fr = new FileReader(fileToProcess); 
        BufferedReader br = new BufferedReader(fr);       
        while ( (lineToRead = br.readLine()) != null) {
          StringTokenizer st = new StringTokenizer(lineToRead);         
          tokenNum = 0;
          while ( st.hasMoreTokens() ) {
        	  st.nextToken();
        	  tokenNum++;
          }
          lineNum++;
       }
    } catch (IOException e) { 
    }
    
    inputMatrix = new double[lineNum][tokenNum];
    lineNum = 0;
    tokenNum = 0;
    
    //Populate matrix with values
    try { 
    	FileReader fr = new FileReader(fileToProcess); 
        BufferedReader br = new BufferedReader(fr); 
        
        while ( (lineToRead = br.readLine()) != null) {
          StringTokenizer st = new StringTokenizer(lineToRead);
          
          tokenNum = 0;
          while ( st.hasMoreTokens() ) {
        	  inputMatrix[lineNum][tokenNum] = Double.parseDouble( st.nextToken() );
        	  //System.out.println(Double.parseDouble( st.nextToken() ));
        	  //System.out.print(inputMatrix[lineNum][tokenNum]);
        	  tokenNum++;
          }

          lineNum++;
       }

    } catch (IOException e) { 

    }

/*
    for (int i = 0; i < lineNum; i++) {
    	for (int j = 0; j < tokenNum; j++) {
    		System.out.print(inputMatrix[i][j] + " ");
    	}
    	System.out.println();
    }
*/
	
	return inputMatrix;
}
                
}
