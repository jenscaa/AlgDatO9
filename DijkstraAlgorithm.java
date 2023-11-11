package AlgDatO9;

import java.io.File;
import java.io.IOException;
import java.util.*;

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
     * @param fromNode The starting node of the edge.
     * @param toNode The ending node of the edge.
     * @param travelTime The time it takes to traverse the edge.
     * @param length The length of the edge.
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

    /**
     * Class constructor.
     */
    public DijkstraAlgorithm() {
        nodes = new HashMap<>();
        allEdges = new HashMap<>();
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
        while(scanner.hasNext()) {
            // The format for a Node-file is: nodeNumber latitude longitude
            int nodeNumber = scanner.nextInt();
            double latitude = Double.parseDouble(scanner.next());
            double longitude = Double.parseDouble(scanner.next());
            Node node = new Node(nodeNumber, latitude, longitude);

            nodes.put(nodeNumber, node);
            allEdges.put(nodeNumber, new ArrayList<>());
        }
        System.out.println("Reading nodes done");
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
        while(scanner.hasNext()) {
            // The format for an Edge-file is: fromNode toNode travelTime length speedLimit
            int fromNode = scanner.nextInt();
            int toNode = scanner.nextInt();
            int travelTime = scanner.nextInt();
            int length = scanner.nextInt();
            int speedLimit = scanner.nextInt();

            Edge edge = new Edge(fromNode, toNode, travelTime, length, speedLimit);
            allEdges.get(fromNode).add(edge);
        }
        System.out.println("Reading edges done");
        scanner.close();
    }

    /**
     * Uses Dijkstra´s algorithm to find the shortest path from a start node to an end node.
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
        // except for the startNode which is set to 0 (logically)
        Map<Integer, Integer> distance = new HashMap<>();
        for (int node : nodes.keySet()) {
            distance.put(node, Integer.MAX_VALUE);
        }
        distance.put(startNode, 0);

        // Initialize the priority queue with the start node.
        // It is used to select the node with the minimum distance for each iteration.
        // It is initialized with a Comparator that prioritizes
        // nodes based on their distance to the starting node.
        // The start node is added to the priority queue initially,
        // since we want to start our search from this node.
        PriorityQueue<Integer> priorityQueue = new PriorityQueue<>(Comparator.comparingInt(distance::get));
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
            dijkstraAlgorithm.readNodeFile("src/AlgDatO9/island.noder.txt");
            dijkstraAlgorithm.readEdgeFile("src/AlgDatO9/island.kanter.txt");

            int startNode = 0;
            int endNode = 1000;
            int shortestDistance = dijkstraAlgorithm.dijkstra(startNode, endNode);

            System.out.println("Shortest distance from Node " + startNode + " to Node " + endNode + ": " + shortestDistance);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}

