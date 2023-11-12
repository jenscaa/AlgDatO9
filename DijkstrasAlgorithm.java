package AlgDatO9;

import java.io.BufferedReader;
import java.io.FileReader;
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
    // Distance from starting Node
    int distanceFromStartNode = Integer.MAX_VALUE;

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
    // The time it takes to traverse the edge in seconds.
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
            int amountNodes = Integer.parseInt(stringTokenizer.nextToken());
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
     * @throws IOException If an Input/Output error occurs.
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
     * Uses DijkstraÂ´s algorithm to find the shortest path from a start node to an end node.
     *
     * @param startNode The starting node.
     * @param endNode   The ending node.
     * @return The shortest distance from the start node to the end node. Returns -1 if there is no path.
     */
    public int dijkstra(int startNode, int endNode) {
        // Initialize the distance map.
        // It is used to store the minimum distance
        // from the start node to each other node.
        // Initially, all distances are set to
        // Integer.MAX_VALUE to represent "infinity",
        // except for the startNode which is set to 0.

        Node startingNode = nodes[startNode];
        Node endingNode = nodes[endNode];
        startingNode.distanceFromStartNode = 0;

        // Initialize the priority queue with the start node.
        // It is used to select the node with the minimum distance for each iteration.
        // It is initialized with a Comparator that prioritizes
        // nodes based on their distance to the starting node.
        // The start node is added to the priority queue initially.
        PriorityQueue<Node> priorityQueue = new PriorityQueue<>(Comparator.comparingInt(node -> node.distanceFromStartNode));
        priorityQueue.add(startingNode);

        // Dijkstra's algorithm.
        // For each iteration it selects the node with the lowest
        // distance from the starting node and checks its edges.
        // For each edge, if the new distance is shorter than any
        // of the other edges, then the Node that the edge leads to is added to distances.
        // This will repeat until the PriorityQueue is empty (meaning we got no more Nodes to explore)
        while (!priorityQueue.isEmpty()) {
            Node currentNode = priorityQueue.poll();

            for (Edge edge : currentNode.edges) {
                int newDistance = currentNode.distanceFromStartNode + edge.length;

                // Update the distance if a shorter path is found
                if (newDistance < edge.toNode.distanceFromStartNode) {
                    edge.toNode.distanceFromStartNode = newDistance;
                    priorityQueue.add(edge.toNode);
                }
            }
        }
        // Check if the distance to the endNode is still Integer.MAX_VALUE
        // This means it is not possible to reach the endNode from the startNode
        if (endingNode.distanceFromStartNode == Integer.MAX_VALUE) {
            return -1;
        }
        return endingNode.distanceFromStartNode;
    }

    public static void main(String[] args){
        DijkstrasAlgorithm dijkstras = new DijkstrasAlgorithm();
        dijkstras.readNodeFile("src/AlgDatO9/noder.txt");
        System.out.println("NODES ARE READ FROM FILE");
        dijkstras.readEdgeFile("src/AlgDatO9/kanter.txt");
        System.out.println("EDGES ARE READ FROM FILE");

        int startNode = 7705656;
        int endNode = 2800567;
        Date start = new Date();
        dijkstras.dijkstra(startNode, endNode);
        Date end = new Date();

        double time = (double) end.getTime()-start.getTime();
        System.out.println("Milliseconds used on Dijkstras Algorithm: " + time);
    }
}
