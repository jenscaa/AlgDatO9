import java.io.BufferedWriter;
import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.*;
import java.util.logging.Logger;

/**
 * Class that represents a Node.
 */
class Node {

  // Number that represents the node.
  int nodeNumber;
  /// The latitude coordinate of the node.
  double latitude;
  // The longitude coordinate of the node.
  double longitude;

  /**
   * Class constructor.
   *
   * @param nodeNumber Number that represents the node.
   * @param latitude   The latitude coordinate of the node.
   * @param longitude  The longitude coordinate of the node.
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

  // The starting node of the edge.
  int fromNode;
  // The ending node of the edge.
  int toNode;
  // The time it takes to traverse the edge.
  int travelTime;
  // The length of the edge.
  int length;
  // The speed limit when traversing the edge.
  int speedLimit;

  /**
   * Class constructor.
   *
   * @param fromNode   The starting node of the edge.
   * @param toNode     The ending node of the edge.
   * @param travelTime The time it takes to traverse the edge.
   * @param length     The length of the edge.
   * @param speedLimit The speed limit when traversing the edge.
   */
  public Edge(int fromNode, int toNode, int travelTime, int length, int speedLimit) {
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
public class DijkstraAlgorithm {

  // Map that stores all the Nodes, the key is the nodeNumber.
  private Map<Integer, Node> nodes;
  // Map that stores a list of all the edges for each node, the key is the nodeNumber.
  private Map<Integer, ArrayList<Edge>> allEdges;
  Map<Integer, Integer> distance = new HashMap<>();
  Logger logger = Logger.getLogger(this.getClass().getName());

  /**
   * Class constructor.
   */
  public DijkstraAlgorithm() {
    nodes = new HashMap<>();
    allEdges = new HashMap<>();
  }

  public Map<Integer, Node> getNodes() {
    return nodes;
  }

  public Map<Integer, ArrayList<Edge>> getAllEdges() {
    return allEdges;
  }

  /**
   * Read the file containing information on Nodes.
   *
   * @param fileName The name of the file you want to read.
   * @throws IOException If an Input/Output error occurs.
   */
  public void readNodeFile(String fileName) throws IOException {
    // Use Scanner to read from the file.
    Scanner scanner = new Scanner(new File(fileName));

    // Number of nodes is stored at the beginning of file
    int numNodes = scanner.nextInt();

    // Read the file line-for-line
    while (scanner.hasNext()) {
      // The format for a Node-file is: nodeNumber latitude longitude
      int nodeNumber = scanner.nextInt();
      double latitude = Double.parseDouble(scanner.next());
      double longitude = Double.parseDouble(scanner.next());
      Node node = new Node(nodeNumber, latitude, longitude);

      nodes.put(nodeNumber, node);
      allEdges.put(nodeNumber, new ArrayList<>());
    }
    logger.info("Reading nodes: done");
    scanner.close();
  }

  /**
   * Read the file containing information on Edges.
   *
   * @param fileName The name of the file you want to read.
   * @throws IOException If an Input/Output error occurs.
   */
  public void readEdgeFile(String fileName) throws IOException {
    // Use Scanner to read from the file.
    Scanner scanner = new Scanner(new File(fileName));

    // Number of edges is stored at the beginning of file
    int numEdges = scanner.nextInt();

    // Read the file line-for-line
    while (scanner.hasNext()) {
      // The format for an Edge-file is: fromNode toNode travelTime length speedLimit
      int fromNode = scanner.nextInt();
      int toNode = scanner.nextInt();
      int travelTime = scanner.nextInt();
      int length = scanner.nextInt();
      int speedLimit = scanner.nextInt();

      Edge edge = new Edge(fromNode, toNode, travelTime, length, speedLimit);
      allEdges.get(fromNode).add(edge);
    }
    logger.info("Reading edges: done");
    scanner.close();
  }

  /**
   * Uses Dijkstra´s algorithm to find the shortest path from a start node to an end node.
   *
   * @param startNode The starting node.
   * @param endNode   The ending node.
   * @return The shortest distance from the start node to the end node. Returns -1 if there is no
   * path.
   */
  public int dijkstra(int startNode, int endNode) {
    // Initialize the distance map.
    // It is used to store the minimum distance
    // from the start node to each other node.
    // Initially, all distances are set to
    // Integer.MAX_VALUE to represent "infinity",
    // except for the startNode which is set to 0 (logically)
    for (int node : nodes.keySet()) {
      distance.put(node, Integer.MAX_VALUE);
    }
    distance.put(startNode, 0);

    // Initialize the priority queue with the start node
    // since we want to start our search from this node.
    // It is set up to prioritize nodes based on their distances from the start node.
    // This is achieved with the Comparator.comparingInt(distance::get).
    // The comparator compares nodes based on their distances in the distance map
    // which gets updated in while-loop below.
    PriorityQueue<Integer> priorityQueue
        = new PriorityQueue<>(Comparator.comparingInt(distance::get));
    priorityQueue.add(startNode);

    // Dijkstra's algorithm.
    // For each iteration it selects the node with the lowest
    // distance from the starting node and checks its edges.
    // For each edge, if the new distance is shorter than any
    // of the other edges, then the Node that the edge leads to is added to PriorityQueue in addition.
    // This will repeat until the PriorityQueue is empty (meaning we got no more Nodes to explore)
    while (!priorityQueue.isEmpty()) {
      int currentNode = priorityQueue.poll();

      // Check if the current node is the end node
      if (currentNode == endNode) {
        // Reached the end node
        break;
      }

      for (Edge edge : allEdges.get(currentNode)) {
        int newDist = distance.get(currentNode) + edge.length;

        // Update the distance if a shorter path is found
        // Here we also bypass the problem with PriorityQueue
        // not allowing us to directly change priorities, by
        // re-adding the node with the new priority after having it removed
        // and then updating the value in distance to reflect this change in priority.
        if (newDist < distance.get(edge.toNode)) {
          distance.put(edge.toNode, newDist);
          priorityQueue.add(edge.toNode);
        }
      }
    }
    // Check if the distance to the endNode is still Integer.MAX_VALUE
    // This means it is not possible to reach the endNode from the startNode
    if (distance.get(endNode) == Integer.MAX_VALUE) {
      return -1;
    }
    return distance.get(endNode);
  }

  public static void main(String[] args) {
    try {
      DijkstraAlgorithm dijkstraAlgorithm = new DijkstraAlgorithm();
      dijkstraAlgorithm.readNodeFile("island.noder.txt");
      dijkstraAlgorithm.readEdgeFile("island.kanter.txt");

      int startNode = 0;
      int endNode = 1000;
      int shortestDistance = dijkstraAlgorithm.dijkstra(startNode, endNode);

      System.out.println("Shortest distance from Node " + startNode +
          " to Node " + endNode + ": " + shortestDistance);
    } catch (IOException e) {
      e.printStackTrace();
    }
  }
}

/**
 * Class representing the preprocessed data of dijkstra algorithm.
 * <br>Uses the dijkstra algorithm to find the shortest distances from nodes in a graph to a
 * select few landmark nodes.
 */
class PreprocessedDijkstra extends DijkstraAlgorithm {
  Map<Integer, Integer> distance = new HashMap<>();
  //The map of preprocessed data functions much like a two-dimensional array. Used a map instead
  // of int[][] since we want to store the node numbers of the landmarks in the array AND there
  // will only be a few landmarks in total. Using a regular 2D array would result a very large
  // outer array (int[500000][whatever]) where only a few of the elements are actually in use.
  // This means a lot of unused space and more time spent iterating than necessary. Using a Map
  // avoids this.
  Map<Integer, int[]> preprocessedData = new HashMap<>();

  /**
   * Runs the dijkstra algorithm on the whole graph from the start node.
   * @param startNode the node where the algorithm starts.
   */
  private void dijkstra(int startNode) {
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
  }

  /**
   * Preprocesses a graph using the dijkstra algorithm and landmarks as start-nodes.
   * @param landmarks array containing the node numbers of landmarks to run the dijkstra
   *                  algorithm from.
   */
  private void preprocess(int[] landmarks) {
    for (int landmark : landmarks) {
      dijkstra(landmark);
      int[] distanceArray = new int[distance.size()];
      distance.forEach((k, v) -> distanceArray[k] = v);
      preprocessedData.put(landmark, distanceArray);
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
  public void writeFromLandmarks(String fileName, int[] landmarks)
      throws IOException {
    BufferedWriter bufferedWriter = new BufferedWriter(new FileWriter(fileName));
    preprocess(landmarks);


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

  public static void main(String[] args) {
    try {
      PreprocessedDijkstra pd = new PreprocessedDijkstra();
      pd.readNodeFile("island.noder.txt");
      pd.readEdgeFile("island.kanter.txt");
      int[] landmarks = {0};
      String filename = "island-preprossesert-0.txt";
      pd.writeFromLandmarks(filename, landmarks);

    } catch (IOException e) {
      throw new RuntimeException(e);
    }
  }
}



