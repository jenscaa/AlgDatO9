import java.io.BufferedReader;
import java.io.FileReader;
import java.io.IOException;
import java.util.ArrayList;
import java.util.List;
import java.util.StringTokenizer;

/**
 * Represents the solution for <i>IDATT2101 oving 9: ALT & Dijkstra</i>.
 *
 * @author Jens Christian Aanestad
 * @version 12.10.2023
 * @see Edge
 * @see Node
 * @see Predecessor
 */
public class Graph {
  int nodes;
  int edges;
  Node[] nodeArray;
  
  /**
   * Reads and initializes nodes from the stream of a BufferedReader
   * from a file. This assumes that the file is on the following format:
   * <pre>
   * 7956886                      (Number of nodes)
   * 0 55.6345298 12.0729630      (Node number, latitude, and longitude)
   * 1 55.6345880 12.0722614      --||--
   * 2 55.6346358 12.0705787      --||--
   * 3 55.6390613 12.0686169      --||--
   * </pre>
   *
   * @param fileLocation the file location of the file to read.
   * @throws IOException if file/file-format is wrong.
   * @see java.io.StreamTokenizer
   */
  private void readNodes(String fileLocation) throws IOException {
    try (BufferedReader bufferedReader = new BufferedReader(new FileReader(fileLocation))) {
      StringTokenizer st = new StringTokenizer(bufferedReader.readLine());
      nodes = Integer.parseInt(st.nextToken());
      nodeArray = new Node[nodes];
      for (int i = 0; i < nodes; ++i) {
        st = new StringTokenizer(bufferedReader.readLine());
        int nodeNumber = Integer.parseInt(st.nextToken());
        double latitude = Double.parseDouble(st.nextToken());
        double longitude = Double.parseDouble(st.nextToken());
        nodeArray[i] = new Node(nodeNumber, latitude, longitude);
      }
    }
    catch (IOException e) {
      e.printStackTrace();
    }
  }
  
  /**
   * Reads and initializes edges from the stream of a BufferedReader
   * from a file. This assumes that the nodes has been initialized and
   * that the file is on the following format:
   *
   * <pre>
   * 17815613         (Number of edges)
   * 0 1 792          (From node, to node, drive time (in hundredths of a second))
   * 1 0 792          --||--
   * 1 2 1926         --||--
   * 2 1 1926         --||--
   * </pre>
   *
   * @param fileLocation the file location of the file to read.
   * @throws IOException if file/file-format is wrong.
   * @see java.io.StreamTokenizer
   */
  private void readEdges(String fileLocation) throws IOException {
    try (BufferedReader bufferedReader = new BufferedReader(new FileReader(fileLocation))) {
      StringTokenizer st = new StringTokenizer(bufferedReader.readLine());
      edges = Integer.parseInt(st.nextToken());
      for (int i = 0; i < nodes; ++i) {
        st = new StringTokenizer(bufferedReader.readLine());
        int startNode = Integer.parseInt(st.nextToken());
        int endNode = Integer.parseInt(st.nextToken());
        int driveTime = Integer.parseInt(st.nextToken());
        nodeArray[startNode].headEdge = new Edge(nodeArray[startNode], nodeArray[endNode], nodeArray[startNode].headEdge, driveTime);
      }
    }
    catch (IOException e) {
      e.printStackTrace();
    }
  }
  
  /**
   * Initializes a predecessor for every node in the node array,
   * and sets all distances to infinity except the start node,
   * which distance is set to 0. This is done for a BFS.
   *
   * @param startNode the start node.
   */
  public void initPredecessor(Node startNode) {
    for (int i = nodes; i-- > 0;) {
      nodeArray[i].data = new Predecessor();
    }
    ((Predecessor)startNode.data).distance = 0;
  }
  
  /**
   * Returns the edge between given nodes. This method assumes that there are only
   * edge in one direction between two nodes.
   *
   * @param fromNode the node at the start of the edge.
   * @param toNode the node at the end of the edge.
   * @return Edge. If not found, then returns null.
   */
  public Edge getEdge(Node fromNode, Node toNode) {
    for (Edge edge = fromNode.headEdge; edge != null; edge = edge.nextEdge) {
      if (edge.endNode == toNode) {
        return edge;
      }
    }
    return null;
  }
  
  /**
   * Checks if edge between two given nodes exists.
   *
   * @param fromNode the node at the start of the edge.
   * @param toNode the node at the end of the edge.
   * @return true if exists.
   */
  public boolean edgeExists(Node fromNode, Node toNode) {
    return getEdge(fromNode, toNode) != null;
  }
  
  /**
   * Deletes an edge between two given nodes. This method assumes that the edge does exist.
   *
   * @param fromNode the node at the start of the edge.
   * @param toNode the node at the end of the edge.
   */
  public void deleteEdge(Node fromNode, Node toNode) {
    Edge preEdge = null;
    Edge currentEdge = fromNode.headEdge;
    while (currentEdge != null && currentEdge!= getEdge(fromNode, toNode)) {
      preEdge = currentEdge;
      currentEdge = currentEdge.nextEdge;
    }
    if (currentEdge != null) {
      if (preEdge != null) preEdge.nextEdge = currentEdge.nextEdge;
      else fromNode.headEdge = currentEdge.nextEdge;
      currentEdge.nextEdge = null;
    }
  }
  
  /**
   * Adds an edge to the graph between two given nodes with a flow value.
   *
   * @param fromNode the node at the start of the edge.
   * @param toNode the node at the end of the edge.
   * @param value the flow value of this edge.
   */
  public void addEdge(Node fromNode, Node toNode, Edge nextEdge, int value) {
    nodeArray[fromNode.number].headEdge = new Edge(nodeArray[fromNode.number], nodeArray[toNode.number]
            , nodeArray[fromNode.number].headEdge, value);
  }
  
  /**
   * Main method - Entry point
   *
   * @param args String[]
   */
  public static void main(String[] args) throws IOException {
    
    try {
      Graph graph = new Graph();
      graph.readNodes("smallTestNodes.txt");
      graph.readEdges("smallTestEdges.txt");
  
      for (Node n : graph.nodeArray) {
        System.out.println("Node: " + n.number + ", " +n.latitude + ", " +n.latitude);
        for (Edge e = n.headEdge; e!= null; e = e.nextEdge) {
          System.out.println("    Edge: " + e.startNode.number + ", " + e.endNode.number + ", " + e.nextEdge + ", " + e.driveTime);
        }
      }
    } catch (IOException e) {
      e.printStackTrace();
    }
    
  }
}

/**
 * Represents an edge as a linked-list in a graph used for <i>IDATT2101 oving 9</i>.
 *
 * @author Jens Christian Aanestad
 * @version 04.10.2023
 */
class Edge {
  Node startNode;
  Node endNode;
  Edge nextEdge;
  int driveTime;
  
  /**
   * Initializes an edge from <i>node</i> with the next edge.
   *
   * @param startNode the node this edge comes from.
   * @param endNode the node this edge points to.
   * @param nextEdge the next edge from this edge.
   * @param driveTime the value of this edge.
   */
  public Edge(Node startNode, Node endNode, Edge nextEdge, int driveTime) {
    this.startNode = startNode;
    this.endNode = endNode;
    this.nextEdge = nextEdge;
    this.driveTime = driveTime;
  }
}
/**
 * Represents a node in a graph used for <i>IDATT2101 oving 9</i>.
 *
 * @author Jens Christian Aanestad
 * @version 04.10.2023
 */
class Node {
  Edge headEdge;
  int number;
  double latitude;
  double longitude;
  Object data;
  
  /**
   * Initializes a node with a number.
   *
   * @param number the node's number.
   * @param latitude the node's latitude.
   * @param longitude the node's longitude.
   */
  public Node(int number, double latitude, double longitude) {
    this.number = number;
    this.latitude = latitude;
    this.longitude = longitude;
  }
}

/**
 * Represents a predecessor for a BFS algorithm used for <i>IDATT2101 oving 7</i>.
 *
 * @author Jens Christian Aanestad
 * @version 04.10.2023
 */
class Predecessor {
  int distance;
  Node predecessor;
  final static int INFINITE = 1_000_000_000;
  
  /**
   * Initializes a predecessor by setting the distance to infinite.
   */
  public Predecessor() {
    distance = INFINITE;
  }
}