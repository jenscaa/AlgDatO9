package AlgDatO9;

import java.io.BufferedReader;
import java.io.FileReader;
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
        // Initially, all nodes have their travel time to the start node set to
        // Integer.MAX_VALUE to represent "infinity",
        // except for the startNode which is set to 0.
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
