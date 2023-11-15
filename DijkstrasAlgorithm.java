import java.io.*;
import java.util.*;
import java.util.logging.Logger;

import static java.util.Comparator.comparingInt;

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
    // Estimate for the node calculated from landmarks.
    int estimate = 0;
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
    protected Node[] nodes;
    // Array that stores all the Nodes with inverted edges based on their nodeNumber
    protected Node[] invertedNodes;

    protected int amountNodes;

    //The map of preprocessed data functions much like a two-dimensional array. Used a map instead
    // of int[][] since we want to store the node numbers of the landmarks in the array AND there
    // will only be a few landmarks in total. Using a regular 2D array would result a very large
    // outer array (int[500000][whatever]) where only a few of the elements are actually in use.
    // This means a lot of unused space and more time spent iterating than necessary. Using a Map
    // avoids this.
    protected Map<Integer, int[]> landmarkToNodes = new HashMap<>();

    protected Map<Integer, int[]> nodesToLandmark = new HashMap<>();

    Logger logger = Logger.getLogger(this.getClass().getName());
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
            invertedNodes = new Node[amountNodes];
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
                invertedNodes[nodeInverted.nodeNumber] = nodeInverted;
            }
            logger.info("DONE READING: " + fileName);
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
            logger.info("DONE READING: " + fileName);
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
                Node fromNode = invertedNodes[Integer.parseInt(stringTokenizer.nextToken())];
                Node toNode = invertedNodes[Integer.parseInt(stringTokenizer.nextToken())];
                int travelTime = Integer.parseInt(stringTokenizer.nextToken());
                int length = Integer.parseInt(stringTokenizer.nextToken());
                int speedLimit = Integer.parseInt(stringTokenizer.nextToken());

                // Edge has toNode and fromNode swapped (to invert the table)
                Edge edge = new Edge(toNode, fromNode, travelTime, length, speedLimit);
                invertedNodes[edge.fromNode.nodeNumber].edges.add(edge);
            }
            logger.info("DONE READING: " + fileName);
        } catch (Exception e){
            e.printStackTrace();
        }
    }

    /**
     * Read the file containing information on
     * the points of interests. Get a Map that contains
     * every point of interest of a specific type.
     *
     * @param fileName The name of the file you want to read.
     * @param type Integer representing the type of node you want.
     * @return Map containing every point of a specific type.
     */
    public Map<Integer, Node> readPointOfInterestFile(String fileName, int type) {
        // Map that contains the node of the wanted type
        Map<Integer, Node> nodeMap = new HashMap<>();
        try {
            // Using BufferedReader and StringTokenizer to read from file, should be the most optimal.
            BufferedReader bufferedReader = new BufferedReader(new FileReader(fileName));
            StringTokenizer stringTokenizer = new StringTokenizer(bufferedReader.readLine());
            // The amount of edges is on the first line of the file.
            int Interessepekt = Integer.parseInt(stringTokenizer.nextToken());
            for (int i = 0; i < Interessepekt; i++) {
                // Read the file line-for-line.
                stringTokenizer = new StringTokenizer(bufferedReader.readLine());
                // The format for an Edge-file is: fromNode toNode travelTime length speedLimit
                int nodeNumber = Integer.parseInt(stringTokenizer.nextToken());
                int nodeType = Integer.parseInt(stringTokenizer.nextToken());
                String name = stringTokenizer.nextToken();
                if ((nodeType & type) == type) {
                    nodeMap.put(nodeNumber, nodes[nodeNumber]);
                }
            }
        } catch (Exception e) {
            e.printStackTrace();
        }
        return nodeMap;
    }


    /**
     * Uses Dijkstra´s algorithm to find the shortest path from a start node to an end node.
     *
     * @param startNodeNumber The starting node´s node number.
     * @param endNodeNumber   The ending node´s node number.
     * @param nodeArray       Array of nodes that will be searched through.
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
        PriorityQueue<Node> priorityQueue = new PriorityQueue<>(comparingInt(node -> node.travelTimeFromStartNode));
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
                // after having it removed (if it already is in the queue)
                if (newTravelTime < edge.toNode.travelTimeFromStartNode) {
                    priorityQueue.remove(edge.toNode);
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
     * Uses Dijkstra´s algorithm to find the points of interest
     * that are the closest to the starting node.
     *
     * @param startNodeNumber The starting node´s node number.
     * @param nodeArray       The array of nodes that will be searched through
     * @param nodeMapPointOfInterest         Map containg the nodes with the wanted point of interest
     * @param amountOfPointsOfInterests      Integer representing how many points of interest you want to find
     * @return List of the points of interests, as Nodes, that are closest to the starting node, the amount is specified.
     */
    public List<Node> dijkstraFindPointOfInterest(int startNodeNumber, Node[] nodeArray, Map<Integer, Node> nodeMapPointOfInterest, int amountOfPointsOfInterests) {
        Node startNode = nodeArray[startNodeNumber];
        Node endNode = nodeArray[nodeArray.length - 1];
        List<Node> nodesOfInterests = new ArrayList<>();

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

            if (nodeMapPointOfInterest.containsKey(currentNode.nodeNumber)) {
                nodesOfInterests.add(currentNode);
                if(nodesOfInterests.size() == amountOfPointsOfInterests) {
                    break;
                }
            }

            for (Edge edge : currentNode.edges) {
                int newTravelTime = currentNode.travelTimeFromStartNode + edge.travelTime;
                // Update the travel time if a quicker path is found.
                // Here we also bypass the problem with PriorityQueue
                // not allowing us to directly change priorities, by
                // re-adding the node with the new priority (travel time from starting node)
                // after having it removed (if it already is in the queue)
                if (newTravelTime < edge.toNode.travelTimeFromStartNode) {
                    priorityQueue.remove(edge.toNode);
                    edge.toNode.travelTimeFromStartNode = newTravelTime;
                    edge.toNode.previousNode = currentNode;
                    priorityQueue.add(edge.toNode);
                }
            }
        }
        // Print out time it took and number of nodes picked out of the queue
        long endTime = System.currentTimeMillis();
        long executionTime = (endTime - startTime);
        System.out.println("Dijkstra's Algorithm from Node: " + startNodeNumber + " to surrounding points of interest");
        System.out.println("Execution time in milliseconds: " + executionTime);
        System.out.println("Amount of processed nodes: " + nodesPicked);

        return nodesOfInterests;
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

        String edgeFile = "kanter.txt";
        dijkstras.readEdgeFile(edgeFile);

        int startNode = 5009309 ;
        int endNode = 999080;
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

        /*
        for(int i=0; i < shortestPath.size(); i+=38) {
            System.out.println(shortestPath.get(i).latitude + "," + shortestPath.get(i).longitude);
        }
         */
    }
}

class PreprocessedDijkstra extends DijkstrasAlgorithm {

    /**
     * Runs the dijkstra algorithm on the whole graph from the start node.
     * @param startNodeNumber the node where the algorithm starts.
     */
    private void dijkstra(int startNodeNumber, Node[] nodes) {
        Node startNode = nodes[startNodeNumber];
        startNode.travelTimeFromStartNode = 0;
        PriorityQueue<Node> priorityQueue = new PriorityQueue<>(comparingInt(node -> node.travelTimeFromStartNode));
        priorityQueue.add(startNode);
        while (!priorityQueue.isEmpty()) {
            Node currentNode = priorityQueue.poll();
            for (Edge edge : currentNode.edges) {
                int newTravelTime = currentNode.travelTimeFromStartNode + edge.travelTime;
                if (newTravelTime < edge.toNode.travelTimeFromStartNode) {
                    priorityQueue.remove(edge.toNode);
                    edge.toNode.travelTimeFromStartNode = newTravelTime;
                    edge.toNode.previousNode = currentNode;
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

            dijkstra(landmark, nodes);
            int[] landmarkToNodesDriveTime = new int[nodes.length];
            for (Node n : nodes) {
                landmarkToNodesDriveTime[n.nodeNumber] = n.travelTimeFromStartNode;
            }
            landmarkToNodes.put(landmark, landmarkToNodesDriveTime);

            dijkstra(landmark, invertedNodes);
            int[] nodesToLandmarkDriveTime = new int[invertedNodes.length];
            for (Node n : invertedNodes) {
                nodesToLandmarkDriveTime[n.nodeNumber] = n.travelTimeFromStartNode;
            }
            nodesToLandmark.put(landmark, nodesToLandmarkDriveTime);

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

        bufferedWriter.write(String.valueOf(landmarks.length)); bufferedWriter.newLine();
        landmarkToNodes.forEach((k, v) -> {
            try {
                bufferedWriter.write(k + " " + v.length);
                for (int i = 0; i < v.length; i++) {
                    String out = "";
                    out = out.concat(k + " " + i + " " + v[i]);
                    bufferedWriter.newLine(); bufferedWriter.write(out);
                }
                for (int i = 0; i < v.length; i++) {
                    String out = "";
                    out = out.concat(i + " " + k + " " + nodesToLandmark.get(k)[i]);
                    bufferedWriter.newLine(); bufferedWriter.write(out);
                }
                bufferedWriter.newLine(); bufferedWriter.newLine();
            } catch (IOException e) {
                throw new RuntimeException(e);
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
    public void readFromLandmarks(String filename, Map<Integer, int[]> landmarkToNodes, Map<Integer, int[]> nodesToLandmark) {
        try (BufferedReader bufferedReader = new BufferedReader(new FileReader(filename))) {
            StringTokenizer stringTokenizer = new StringTokenizer(bufferedReader.readLine(), " ");

            // Reads the first integer representing amounts of landmarks.
            int landmarks = Integer.parseInt(stringTokenizer.nextToken());
            // Loops for each landmark.
            for (int i = 0; i < landmarks; i++) {
                stringTokenizer = new StringTokenizer(bufferedReader.readLine(), " ");
                int landmark = Integer.parseInt(stringTokenizer.nextToken());
                int arraySize = Integer.parseInt(stringTokenizer.nextToken());
                landmarkToNodes.put(landmark, new int[arraySize]);
                nodesToLandmark.put(landmark, new int[arraySize]);
                //int[] landmarkToNodesArray = new int[arraySize];
                //int[] nodesToLandmarkArray = new int[arraySize];

                String nextLine = bufferedReader.readLine();
                for (int j = 0; j < arraySize; j++) {
                    stringTokenizer = new StringTokenizer(nextLine, " ");
                    int l = Integer.parseInt(stringTokenizer.nextToken());
                    int n = Integer.parseInt(stringTokenizer.nextToken());
                    int minDriveTime = Integer.parseInt(stringTokenizer.nextToken());
                    landmarkToNodes.get(l)[n] = minDriveTime;
                    nextLine = bufferedReader.readLine();
                }
                for (int j = 0; j < arraySize; j++) {
                    stringTokenizer = new StringTokenizer(nextLine, " ");
                    int n = Integer.parseInt(stringTokenizer.nextToken());
                    int l = Integer.parseInt(stringTokenizer.nextToken());
                    int minDriveTime = Integer.parseInt(stringTokenizer.nextToken());
                    nodesToLandmark.get(l)[n] = minDriveTime;
                    nextLine = bufferedReader.readLine();
                }
                //landmarkToNodes.put(landmark, landmarkToNodesArray);
                //nodesToLandmark.put(landmark, nodesToLandmarkArray);
            }
            logger.info("DONE READING: " + filename);
        }catch (IOException e) {
            e.printStackTrace();
        }
    }

    /**
     * Main method - Run this to generate preprocessed file.
     *
     * @param args String[].
     */
    public static void main(String[] args) {
        try {
            PreprocessedDijkstra pd = new PreprocessedDijkstra();
            pd.readNodeFile("noder.txt");
            pd.readEdgeFile("kanter.txt");
            pd.readEdgeFileInverted("kanter.txt");
            String filename = "preprossesert.txt";
            int[] landmarks = new int[]{1102516, 3047524, 4392562, 6101939};
            pd.writeFromLandmarks(filename, landmarks);
        } catch (Exception e) {
            throw new RuntimeException(e);
        }
    }
}

class AltAlgorithm extends DijkstrasAlgorithm {

    /**
     * Returns estimate from behind start.
     *
     * @param n the current node number.
     * @param L the landmark node number.
     * @param goal the end node number.
     * @return estimate.
     */
    public int behindStart(int n, int L, int goal) {
        // If L is after the goal, then the estimate is negative.
        return landmarkToNodes.get(L)[goal] - landmarkToNodes.get(L)[n];
    }

    /**
     * Returns estimate from behind start.
     *
     * @param n the current node number.
     * @param L the landmark node number.
     * @param goal the end node number.
     * @return estimate.
     */
    public int afterGoal(int n, int L, int goal) {
        // If L is behind the start, then the estimate is negative.
        return nodesToLandmark.get(L)[n] - nodesToLandmark.get(L)[goal];
    }

    /**
     * Returns the highest estimate from all landmarks.
     *
     * @param currentNode the current node number.
     * @param endNode the end node number.
     * @return highest estimate.
     */
    public int estimate(int currentNode, int endNode) {
        int highestEstimate = 0;
        for (int landMark : nodesToLandmark.keySet()) {
            int afterGoal = afterGoal(currentNode, landMark, endNode);
            int behindStart = behindStart(currentNode, landMark, endNode);

            if (afterGoal > highestEstimate) {
                highestEstimate = afterGoal;
            }
            if (behindStart > highestEstimate) {
                highestEstimate = behindStart;
            }
        }
        return highestEstimate;
    }

    /**
     * Represents the ALT for finding the shortest drive time between two points.
     *
     * @param startNodeNumber the start node's number.
     * @param endNodeNumber the end node's number.
     * @return the shortest drive time between two points.
     */
    public int ALT(int startNodeNumber, int endNodeNumber) {
        Node startNode = nodes[startNodeNumber];
        Node endNode = nodes[endNodeNumber];
        startNode.travelTimeFromStartNode = 0;
        startNode.estimate = estimate(startNodeNumber, endNodeNumber);

        // PriorityQueue that priorities lowest travelTime + estimate
        PriorityQueue<Node> priorityQueue = new PriorityQueue<>(comparingInt((node) -> node.travelTimeFromStartNode + node.estimate));
        priorityQueue.add(startNode);

        int nodesPicked = 0;
        long startTime = System.currentTimeMillis();

        while (!priorityQueue.isEmpty()) {
            Node currentNode = priorityQueue.poll();
            nodesPicked++;

            for (Edge edge : currentNode.edges) {
                int newTravelTime = currentNode.travelTimeFromStartNode + edge.travelTime;
                edge.toNode.estimate = estimate(edge.toNode.nodeNumber, endNodeNumber);
                if (newTravelTime < edge.toNode.travelTimeFromStartNode) {
                    priorityQueue.remove(edge.toNode);
                    edge.toNode.travelTimeFromStartNode = newTravelTime;
                    edge.toNode.previousNode = currentNode;
                    priorityQueue.add(edge.toNode);
                }
            }
            if (currentNode.equals(endNode)) {
                break;
            }
        }
        long endTime = System.currentTimeMillis();
        long executionTime = (endTime - startTime);
        System.out.println("Alt Algorithm from Node: " + startNodeNumber + " to Node: " + endNodeNumber);
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
     * Main method - entrypoint.
     *
     * @param args String[].
     */
    public static void main(String[] args) {

        PreprocessedDijkstra pd = new PreprocessedDijkstra();
        AltAlgorithm altAlgorithm = new AltAlgorithm();
        pd.readFromLandmarks("preprossesert.txt", altAlgorithm.landmarkToNodes, altAlgorithm.nodesToLandmark);

        String nodeFile = "noder.txt";
        altAlgorithm.readNodeFile(nodeFile);

        String edgeFile = "kanter.txt";
        altAlgorithm.readEdgeFile(edgeFile);

        int startNode = 5009309;
        int endNode = 999080;
        int travelTime = altAlgorithm.ALT(startNode, endNode) / 100; // Divide by 100 to convert it to seconds (from centiseconds)
        List<Node> shortestPath = altAlgorithm.getPath(endNode, altAlgorithm.nodes);
        if (!shortestPath.isEmpty()) {
            System.out.println("The shortest path contains this amount of nodes: " + shortestPath.size());
        } else {
            System.out.println("No path found.");
        }

        int travelTimeHours = travelTime / 3600;
        int travelTimeMinutes = (travelTime % 3600) / 60;
        int travelTimeSeconds = (travelTime - travelTimeHours * 3600 - travelTimeMinutes * 60);
        System.out.println("The shortest path takes this amount of time: " + travelTimeHours + " hour(s), " + travelTimeMinutes + " minute(s) and " + travelTimeSeconds + " second(s)\n");

        /*
        for (int i = 0; i < shortestPath.size(); i+=38) {
            System.out.println(shortestPath.get(i).latitude + "," + shortestPath.get(i).longitude);
        }
         */
    }
}