import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.util.*;
/**
 * Class that represents a Node.
 */
class Node {
    // Number that represents the node.
    int nodeNumber;
    // The latitude coordinate of the node.
    double latitude;
    // The longitude coordinate of the node.
    double longitude;
    // List of all the edges from the Node
    List<Edge> edges = new ArrayList<>();
    // Travel time from starting Node, initially set to "infinite"
    int travelTimeFromStartNode = Integer.MAX_VALUE;
    // Previous node in the shortest path
    Node previousNode = null;

    /**
     * Class constructor.
     *
     * @param nodeNumber Number that represents the node.
     * @param latitude The latitude coordinate of the node.
     * @param longitude The longitude coordinate of the node.
     */
    public Node(int nodeNumber, double latitude, double longitude) {
        this.nodeNumber = nodeNumber;
        this.latitude = latitude;
        this.longitude = longitude;
    }
}

/**
 * Class that represents an edge between two Nodes
 */
class Edge {
    // The node the edge is going from.
    Node fromNode;
    // The node the edge is going to.
    Node toNode;
    // The time it takes to traverse the edge in centi-seconds.
    int travelTime;
    // The length of the edge in meters.
    int length;
    // The speed limit when traversing the edge.
    int speedLimit;

    /**
     * Class constructor.
     *
     * @param fromNode The starting node of the edge.
     * @param toNode The ending node of the edge.
     * @param travelTime The time it takes to traverse the edge.
     * @param length The length of the edge.
     * @param speedLimit The speed limit when traversing the edge.
     */
    public Edge(Node fromNode, Node toNode, int travelTime, int length, int speedLimit) {
        this.fromNode = fromNode;
        this.toNode = toNode;
        this.travelTime = travelTime;
        this.length = length;
        this.speedLimit = speedLimit;
    }
}

/**
 *
 */
public class DijkstrasAlgorithm {
    // Array that stores all the Nodes based on their nodeNumber
    private Node[] nodes;
    private int amountNodes;

    public Node[] getNodes() {
      return nodes;
    }
    /**
     * Read the file containing information on Nodes.
     * This information is added to nodes
     *
     * @param fileName The name of the file you want to read.
     */
    public void readNodeFile(String fileName){
        try{
            // Using BufferedReader and StringTokenizer to read from file, should be the most optimal.
            BufferedReader bufferedReader = new BufferedReader(new FileReader(fileName));
            StringTokenizer stringTokenizer = new StringTokenizer(bufferedReader.readLine());
            // The amount of nodes is on the first line of the file.
            amountNodes = Integer.parseInt(stringTokenizer.nextToken());
            // Set the size of nodes equal to the amount of nodes.
            nodes = new Node[amountNodes];
            for(int i = 0; i < amountNodes; i++){
                // Read the file line-for-line.
                stringTokenizer = new StringTokenizer(bufferedReader.readLine());
                // The format for a line in the Node-file is: nodeNumber latitude longitude.
                int nodeNumber = Integer.parseInt(stringTokenizer.nextToken());
                double latitude = Double.parseDouble(stringTokenizer.nextToken());
                double longitude = Double.parseDouble(stringTokenizer.nextToken());

                Node node = new Node(nodeNumber, latitude, longitude);
                Node nodeInverted = new Node(nodeNumber, latitude, longitude);
                nodes[node.nodeNumber] = node;
            }
        }catch (Exception e){
            e.printStackTrace();
        }
    }

    /**
     * Read the file containing information on Edges.
     *
     * @param fileName The name of the file you want to read.
     */
    public void readEdgeFile(String fileName){
        try{
            // Using BufferedReader and StringTokenizer to read from file, should be the most optimal.
            BufferedReader bufferedReader = new BufferedReader(new FileReader(fileName));
            StringTokenizer stringTokenizer = new StringTokenizer(bufferedReader.readLine());
            // The amount of edges is on the first line of the file.
            int amountEdges = Integer.parseInt(stringTokenizer.nextToken());
            for(int i = 0; i < amountEdges; i++){
                // Read the file line-for-line.
                stringTokenizer = new StringTokenizer(bufferedReader.readLine());
                // The format for an Edge-file is: fromNode toNode travelTime length speedLimit
                Node fromNode = nodes[Integer.parseInt(stringTokenizer.nextToken())];
                Node toNode = nodes[Integer.parseInt(stringTokenizer.nextToken())];
                int travelTime = Integer.parseInt(stringTokenizer.nextToken());
                int length = Integer.parseInt(stringTokenizer.nextToken());
                int speedLimit = Integer.parseInt(stringTokenizer.nextToken());

                Edge edge = new Edge(fromNode, toNode, travelTime, length, speedLimit);
                nodes[edge.fromNode.nodeNumber].edges.add(edge);
            }

        }catch (Exception e){
            e.printStackTrace();
        }
    }

    /**
     * Read the file containing information on Edges.
     * This is for inverting all the Edges.
     *
     * @param fileName The name of the file you want to read.
     */
    public void readEdgeFileInverted(String fileName){
        try{
            // Using BufferedReader and StringTokenizer to read from file, should be the most optimal.
            BufferedReader bufferedReader = new BufferedReader(new FileReader(fileName));
            StringTokenizer stringTokenizer = new StringTokenizer(bufferedReader.readLine());
            // The amount of edges is on the first line of the file.
            int amountEdges = Integer.parseInt(stringTokenizer.nextToken());
            for(int i = 0; i < amountEdges; i++){
                // Read the file line-for-line.
                stringTokenizer = new StringTokenizer(bufferedReader.readLine());
                // The format for an Edge-file is: fromNode toNode travelTime length speedLimit
                Node fromNode = nodes[Integer.parseInt(stringTokenizer.nextToken())];
                Node toNode = nodes[Integer.parseInt(stringTokenizer.nextToken())];
                int travelTime = Integer.parseInt(stringTokenizer.nextToken());
                int length = Integer.parseInt(stringTokenizer.nextToken());
                int speedLimit = Integer.parseInt(stringTokenizer.nextToken());

                // Edge has toNode and fromNode swapped (to invert the table)
                Edge edge = new Edge(toNode, fromNode, travelTime, length, speedLimit);
                nodes[edge.fromNode.nodeNumber].edges.add(edge);
            }
        } catch (Exception e){
            e.printStackTrace();
        }
    }



    /**
     * Uses Dijkstra´s algorithm to find the shortest path from a start node to an end node.
     *
     * @param startNodeNumber The starting node´s node number.
     * @param endNodeNumber   The ending node´s node number.
     * @return The shortest travel time from the start node to the end node in centiseconds. Returns -1 if there is no path.
     */
    public int dijkstra(int startNodeNumber, int endNodeNumber, Node[] nodeArray) {
        Node startNode = nodeArray[startNodeNumber];
        Node endNode = nodeArray[endNodeNumber];

        startNode.travelTimeFromStartNode = 0;

        // The priority queue is used to select the node with the shortest travel time
        // to the starting node for each iteration. The priority queue is initialized
        // with a Comparator that prioritizes nodes based on their travel time to the starting node
        // The start node is added to the priority queue initially (since we start searching from this node)
        PriorityQueue<Node> priorityQueue = new PriorityQueue<>(Comparator.comparingInt(node -> node.travelTimeFromStartNode));
        priorityQueue.add(startNode);

        // Counter for the number of nodes picked from the priority queue
        int nodesPicked = 0;
        // Start time of Dijkstra´s algorithm
        long startTime = System.currentTimeMillis();

        // Dijkstra's algorithm.
        // For each iteration it selects the node with the shortest
        // travel time from the starting node and checks its edges.
        // For each edge, if the new travel time is shorter than any
        // of the other edges, then the Node that the edge leads to
        // has its travel time from start node updated, previous node
        // updated, and it is added to the priority queue.
        // This will repeat until the PriorityQueue is empty (meaning we got no more Nodes to explore)
        // OR if the end node has been removed from the queue
        // since we do not need to search further than the end node.
        while (!priorityQueue.isEmpty()) {
            Node currentNode = priorityQueue.poll();
            nodesPicked++; // Increment the counter for nodes picked from queue

            // Check if the current node is the end node
            // meaning we can stop processing.
            // If we do not do this, then every node will be checked (unnecessary overhead).
            if (currentNode == endNode) {
                break;
            }

            for (Edge edge : currentNode.edges) {
                int newTravelTime = currentNode.travelTimeFromStartNode + edge.travelTime;

                // Update the travel time if a quicker path is found.
                // Here we also bypass the problem with PriorityQueue
                // not allowing us to directly change priorities, by
                // re-adding the node with the new priority (travel time from starting node)
                // after having it removed
                if (newTravelTime < edge.toNode.travelTimeFromStartNode) {
                    edge.toNode.travelTimeFromStartNode = newTravelTime;
                    edge.toNode.previousNode = currentNode;
                    priorityQueue.add(edge.toNode);
                }
            }
        }
        // Print out time it took and number of nodes picked out of the queue
        long endTime = System.currentTimeMillis();
        long executionTime = (endTime - startTime);
        System.out.println("Dijkstra's Algorithm from Node: " + startNodeNumber + " to Node: " + endNodeNumber);
        System.out.println("Execution time in milliseconds: " + executionTime);
        System.out.println("Amount of processed nodes: " + nodesPicked);

        // Check if the travel time to the endNode is still Integer.MAX_VALUE
        // This means it is not possible to reach the endNode from the startNode
        if (endNode.travelTimeFromStartNode == Integer.MAX_VALUE) {
            return -1;
        }
        return endNode.travelTimeFromStartNode;
    }

    /**
     * Make a list containing the nodes in the shortest path found from Dijkstra´s algorithm.
     *
     * @param endNodeNumber the end node´s node number used in Dijkstra´s algorithm
     * @return List of the nodes in the shortest path
     */
    public List<Node> getPath(int endNodeNumber, Node[] nodeArray) {
        List<Node> path = new ArrayList<>();
        Node currentNode = nodeArray[endNodeNumber];
        // We retrace the shortest path found from Dijkstra backwards
        // When the currentNode is null means we have reach the startNode
        // since it does not have a previous node.
        while (currentNode != null) {
            path.add(currentNode);
            currentNode = currentNode.previousNode;
        }
        // We then use Collections.reverse to get the correct order of the path.
        Collections.reverse(path);
        return path;
    }


    public static void main(String[] args) {
        DijkstrasAlgorithm dijkstras = new DijkstrasAlgorithm();

        String nodeFile = "noder.txt";
        dijkstras.readNodeFile(nodeFile);
        System.out.println("DONE READING: " + nodeFile);

        String edgeFile = "kanter.txt";
        //dijkstras.readEdgeFile(edgeFile);
        dijkstras.readEdgeFileInverted(edgeFile);
        System.out.println("DONE READING: " + edgeFile + "\n");

        int startNode = 7826348;
        int endNode = 2948202;
        int travelTime = dijkstras.dijkstra(startNode, endNode, dijkstras.nodes) / 100; // Divide by 100 to convert it to seconds (from centiseconds)
        List<Node> shortestPath = dijkstras.getPath(endNode, dijkstras.nodes);

        if (!shortestPath.isEmpty()) {
            System.out.println("The shortest path contains this amount of nodes: " + shortestPath.size());
        } else {
            System.out.println("No path found.");
        }

        int travelTimeHours = travelTime / 3600;
        int travelTimeMinutes = (travelTime % 3600) / 60;
        int travelTimeSeconds = (travelTime - travelTimeHours * 3600 - travelTimeMinutes * 60);
        System.out.println("The shortest path takes this amount of time: " + travelTimeHours + " hour(s), " + travelTimeMinutes + " minute(s) and " + travelTimeSeconds + " second(s)\n");
    }
}


/**
 * Class representing the preprocessed data of dijkstra algorithm.
 * <br>Uses the dijkstra algorithm to find the shortest distances from nodes in a graph to a
 * select few landmark nodes.
 */
class PreprocessedDijkstra extends DijkstrasAlgorithm {
  //The map of preprocessed data functions much like a two-dimensional array. Used a map instead
  // of int[][] since we want to store the node numbers of the landmarks in the array AND there
  // will only be a few landmarks in total. Using a regular 2D array would result a very large
  // outer array (int[500000][whatever]) where only a few of the elements are actually in use.
  // This means a lot of unused space and more time spent iterating than necessary. Using a Map
  // avoids this.
  Map<Integer, int[]> preprocessedData = new HashMap<>();


  /*private void dijkstra(int startNode) {
    Map<Integer, Node> nodes = super.getNodes();
    Map<Integer, ArrayList<Edge>> allEdges = super.getAllEdges();
    for (int node : nodes.keySet()) {
      distance.put(node, Integer.MAX_VALUE);
    }
    distance.put(startNode, 0);

    PriorityQueue<Integer> priorityQueue
        = new PriorityQueue<>(Comparator.comparingInt(distance::get));
    priorityQueue.add(startNode);

    while (!priorityQueue.isEmpty()) {
      int currentNode = priorityQueue.poll();

      for (Edge edge : allEdges.get(currentNode)) {
        int newDist = distance.get(currentNode) + edge.length;
        if (newDist < distance.get(edge.toNode)) {
          distance.put(edge.toNode, newDist);
          priorityQueue.add(edge.toNode);
        }
      }
    }
  }*/

  private Map<Integer, Integer> dijkstraTravelTime(int startNodeNumber, Node[] nodeArray) {
      Map<Integer, Integer> travelTimeMap = new HashMap<>();
      for (Node node : nodeArray) {
          travelTimeMap.put(node.nodeNumber, Integer.MAX_VALUE);
      }

      Node startNode = nodeArray[startNodeNumber];
      startNode.travelTimeFromStartNode = 0;
      PriorityQueue<Node> priorityQueue = new PriorityQueue<>(Comparator.comparingInt(node -> node.travelTimeFromStartNode));
      priorityQueue.add(startNode);

      while (!priorityQueue.isEmpty()) {
          Node currentNode = priorityQueue.poll();
          for (Edge edge : currentNode.edges) {
              int newTravelTime = currentNode.travelTimeFromStartNode + edge.travelTime;
              if (newTravelTime < edge.toNode.travelTimeFromStartNode) {
                  edge.toNode.travelTimeFromStartNode = newTravelTime;
                  edge.toNode.previousNode = currentNode;
                  priorityQueue.add(edge.toNode);
                  travelTimeMap.put(currentNode.nodeNumber, newTravelTime);
              }
          }
      }
      return travelTimeMap;
  }

  /**
   * Preprocesses a graph using the dijkstra algorithm and landmarks as start-nodes.
   * @param landmarks array containing the node numbers of landmarks to run the dijkstra
   *                  algorithm from.
   */
  private void preprocess(int[] landmarks, Node[] nodes) {
    for (int landmark : landmarks) {
      Map<Integer, Integer> travelMap = dijkstraTravelTime(landmark, nodes);
      int[] travelArray = new int[travelMap.size()];
      travelMap.forEach((k, v) -> travelArray[k] = v);
      preprocessedData.put(landmark, travelArray);
    }
  }

  /**
   * Writes preprocessed dijkstra data to a file with the format:<br>
   * ------------------------------------------------------------------<br>
   * landmark(1)-distArray(1) ...landmark(n)-distArray(n)<br>
   * landmarkNum distIndex distance<br>
   * ------------------------------------------------------------------<br>
   * For example, only one landmark = 0 may result in:
   * ------------------------------------------------------------------<br>
   * 0-112779<br>
   * 0 0 0<br>
   * 0 1 1251<br>
   * 0 2 1422<br>
   * ...<br>
   * ------------------------------------------------------------------
   * @param fileName name of the file containing preprocessed data.
   * @param landmarks array consisting of node numbers, corresponding to landmarks on a map.
   * @throws IOException if there was a problem writing to file.
   */
  public void writeFromLandmarks(String fileName, int[] landmarks, Node[] nodes)
      throws IOException {
    BufferedWriter bufferedWriter = new BufferedWriter(new FileWriter(fileName));
    preprocess(landmarks, nodes);

    StringBuilder distanceSize = new StringBuilder();
    preprocessedData.forEach((k, v) -> distanceSize.append(k.intValue()).append("-").append(v.length).append(" "));
    bufferedWriter.write(distanceSize.toString().trim());

    preprocessedData.forEach((k, v) -> {
      for (int i = 0; i < v.length; i++) {
        String out = "";
        out = out.concat(k + " " + i +" " + v[i]);
        try {
          bufferedWriter.newLine();
          bufferedWriter.write(out);
        } catch (IOException e) {
          throw new RuntimeException(e);
        }
      }
    });
    bufferedWriter.close();
  }

  /**
   * Reads a preprocessed file containing the shortest distance between landmarks and all other
   * nodes in a graph.
   * @param filename name of file to read.
   * @return The {@code Map<Integer, int[]>} containing landmarks, and distances to other nodes.
   * @throws Exception if it failed to read from the file.
   */
  public Map<Integer, int[]> readFromLandmarks(String filename) throws Exception {
    Map<Integer, int[]> landmarksData= new HashMap<>();
    try {
      BufferedReader bufferedReader = new BufferedReader(new FileReader(filename));
      StringTokenizer stringTokenizer = new StringTokenizer(bufferedReader.readLine(), " ");

      //initializes the landmarksData Map (most importantly the array inside the map) by parsing
      // the first line of the file. The format is landmark-arraySize landmark-arraySize ...
      int firstLineCount = stringTokenizer.countTokens()/2;
      for (int i = 0; i < firstLineCount; i++) {
        int landmark = Integer.parseInt(stringTokenizer.nextToken());
        int arraySize = Integer.parseInt(stringTokenizer.nextToken());
        landmarksData.put(landmark, new int[arraySize]);
      }
      String nextLine = bufferedReader.readLine();
      while (nextLine != null) {
        stringTokenizer = new StringTokenizer(nextLine);
        int landmark = Integer.parseInt(stringTokenizer.nextToken());
        int endNode = Integer.parseInt(stringTokenizer.nextToken());
        int minDistance = Integer.parseInt(stringTokenizer.nextToken());
        landmarksData.get(landmark)[endNode] = minDistance;
        nextLine = bufferedReader.readLine();
      }
      bufferedReader.close();
    }catch (Exception e) {
      throw new Exception(e);
    }
    return landmarksData;
  }

  public static void main(String[] args) {
    try {
      PreprocessedDijkstra pd = new PreprocessedDijkstra();
      pd.readNodeFile("island.noder.txt");
      pd.readEdgeFile("island.kanter.txt");
      String filename = "island.preprossesert.txt";
      int[] landmarks = {0};
      pd.writeFromLandmarks(filename, landmarks, pd.getNodes());

      //Map<Integer, int[]> data = pd.readFromLandmarks(filename);

    } catch (Exception e) {
      throw new RuntimeException(e);
    }
  }
}



