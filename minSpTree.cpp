/*   Create a random undirected graph and find the minimum spanning tree.
 *   Thomas Pliakis
 *   January 14, 2023
 */

#include <fstream>
#include <algorithm>
#include <iostream>
#include <sstream>
#include <unistd.h>
#include <limits>
#include <random>
#include <queue>
#include <vector>
#include <iterator>
#include <list>
#include <ctime>

//=============================================================================
// Edge Class
// The objects of this class are the edges of our graph 
// that keep theabegining, the destination of the edge and its weight.
// ============================================================================
class Edge{
    private:
        int beginNode;
        int destinationNode;
        int weight;
    public:
        Edge(int b, int d, int w){beginNode = b, destinationNode = d; weight =w;}
        Edge():beginNode(0),destinationNode(0),weight(0){}
        //~Edge();

        // Methods
        void print(){
            std::cout << "     ";
            std::cout << beginNode << " " << destinationNode << "\tWeight: " << weight << std::endl;
        }
        //  Set and Get methods
        void set_beginNode(int b){beginNode = b;}
        int get_beginNode(){return beginNode;}
        void set_destinationNode(int d){destinationNode = d;}
        int get_destinationNode(){return destinationNode;}
        void set_weight(int w){weight = w;}
        int get_weight(){return weight;}
};

// ==================================================================================================
// Node Class
// The objects of this class are the Nodes of our graph that keep the the id(name) of the node and
// a list with  edges to the other nodes that this node is connected to.
// =================================================================================================
class Node{
    private:
        int name;
        std::list<Edge> edgeList;
    public:
        // Contructors
        Node(int name , std::list<Edge> edgeList){this->name = name;this->edgeList=edgeList;}
        Node():name(0),edgeList(){} 
        //  ~Node(){cout << "Node "<< name << " destroyed"<< endl;}

        //Methods
        void print(){
            std::cout << "Node no: " << name <<  std::endl;

            std::list<Edge>::iterator it=edgeList.begin();
            while(it!=edgeList.end()){
                it->print();
                it++;
            }
        }
        // Set and Get
        void set_name(int n){name = n;}
        int get_name(){return name;}
        // get the weight between this.node and name node
        int get_weight(int name){
            int w=0;
            std::list<Edge>::iterator it = edgeList.begin();
            // While loop to find the edge in the list of edges
            while(it != edgeList.end()){
                if(it->get_destinationNode() == name){
                    return it->get_weight();
                }
                it++;
            }
            return w;
        }
        std::list<Edge> get_edgeList(){return edgeList;};
        int get_edgeList_size(){return edgeList.size();};
        // Adding a edge
        void add_edge(Node y,int w){
            Edge e(this->name,y.get_name(),w); //declare a variable of type edge
            edgeList.push_back(e);  //add it to edgelist
        }
        // check if an edge exists
        bool check_edge(int n){ 
            bool r=false;
            std::list<Edge>::iterator it=edgeList.begin();
            while(it != edgeList.end()){
                if(it->get_destinationNode() == n){
                    r = true;
                    return r;
                }
                it++;
            }
            return r;
        }
};

// ====================================================================================================================
// Graph Class
// The objects of this class are our graphs. Each graph has a vector of nodes and the numbers of nodes and edges.
// ====================================================================================================================
class Graph{
    private:
        int VG=0;
        int EG=0;
        std::vector<Node> nodes;
    public:
        //Contructors
        Graph():VG(0),EG(0){}
        // The random graph procedure (in the constructor here) have edge density and distance range as parameters.
        Graph(std::string file_name){

            // Read file
            std::ifstream graph_file(file_name);
            // num if the first line, the number of nodes
            int num;
            // Here each line is temporary saved before going on the graph.
            int node1, node2 , weight;

            std::string line;
            std::getline(graph_file, line);
            std::istringstream iss(line);
            iss >> num;
            std::cout << "number of nodes: " << num << std::endl;
            // Read all lines and initiate the graph.
            while (std::getline(graph_file, line))
            {
                std::istringstream iss(line);
                if (!(iss >> node1 >> node2 >> weight)) { break; } // error

                add_Node(node1);
                add_Node(node2);
                add(get_Node(node1),get_Node(node2),weight);
            }
        }
        //~Graph(){cout << "Graph destroyed" << endl;}

        // Methods
        std::vector<Node> getVecOfNodes(){
            return nodes;
        }
        // Get a specific node from the graph
        Node get_Node(int n){
            Node x;
            std::vector<Node>::iterator ve;
            for(ve=nodes.begin(); ve!=nodes.end(); ve++){
                if(ve->get_name() == n){
                    x = *ve;
                    return x;
                }
            }
            return x;
        }
        // Get the weight between 2 nodes
        int get_weight(Node n, Node m){
            int v=0;
            std::vector<Node>::iterator ve;
            for(ve=nodes.begin(); ve!=nodes.end(); ve++){
                if(ve->get_name() == n.get_name()){
                    return ve->get_weight(m.get_name());
                }
            }
            return v;
        }
        // Check if a node already exists in the graph.
        bool node_exists(int n){
            std::vector<Node>::iterator ndit;
            for(ndit=nodes.begin(); ndit!=nodes.end(); ndit++){
                if(ndit->get_name() == n){
                    return true;
                }
            }
            return false;
        }
        // Add a node in our graph
        void add_node(Node n){
            bool nExists = false;
            std::vector<Node>::iterator ndit;
            for(ndit=nodes.begin(); ndit!=nodes.end(); ndit++){
                if(ndit->get_name() == n.get_name()){
                    nExists = true;
                }
            }
            if(!nExists){
                nodes.push_back(n);
                VG++;
            }
        }
        // Add a node in our graph by initializing only its id(name)
        void add_Node(int i){
            Node n;
            n.set_name(i);
            add_node(n);
        }
        // Print our graph
        void print_Graph(){
            std::vector<Node>::iterator it = nodes.begin();
            while(it != nodes.end()){
                it->print();
                it++;
            }
        }
        // Add an edge between 2 nodes
        void add(Node x, Node y, int w){
            std::vector<Node>::iterator it = nodes.begin();
            while(it != nodes.end()){
                if(it->get_name() == x.get_name()){
                    if(!it->check_edge(y.get_name())){
                        it->add_edge(y,w);
                    }
                }
                if(it->get_name() == y.get_name()){
                    if(!it->check_edge(x.get_name())){
                        it->add_edge(x,w);
                    }
                }
                it++;
            }
        }
        // Set and Get path lengths
        // Get the list of edges that a node in our graph has.
        std::list<Edge>  get_edgeList(Node n){
            std::list<Edge> eli;
            std::vector<Node>::iterator ve;
            for(ve=nodes.begin(); ve!=nodes.end(); ve++){
                if(ve->get_name() == n.get_name()){
                    return ve->get_edgeList();
                }
            }
            return eli;
        }
        // Set and get the numbers of nodes and edges.
        int get_VG(){return VG;}
        int get_EG(){return EG;}
};

// ====================================================================================================================
// Priority Class
// The class of our Priority Queue which contains a list of edges.
// The edge at the top of the queue always has the minimum weight and so its the next to picked.
// If 2 edges have the same weight then higher is the one that was entered first.
// ====================================================================================================================
class PriorityQueue {
    private:
        std::list<Edge> queue;

    public:
        // Constructor
        PriorityQueue();
        PriorityQueue(std::list<Edge> queue){this->queue = queue;}
        //~PriorityQueue(){cout << "PQ destroyed" << endl;}

        //Methods
        // Set and Get
        Edge get_top(){
            std::list<Edge>::iterator it = queue.begin() ;
            Edge n = queue.front();
            queue.erase(it);
            return n;
        }
        int get_size(){return queue.size();}
        // Insert an edge to the queue based on its weight, the lowest the weight the higher in the queue.
        void insert(Edge queue_element){
            std::list<Edge>::iterator it = queue.begin();
            bool inserted = false;
            while(it != queue.end()){
                if( queue_element.get_weight() < it->get_weight()){
                    queue.insert(it,queue_element);
                    inserted = true;
                    break;
                }
                it++;
            }
            if(!inserted)
                queue.push_back(queue_element);
        }
        // Check if a node is contained in the queue.
        bool contains(int b, int d){
            std::list<Edge>::iterator it;
            bool contained = false;
            while(it != queue.end()){
                if(it->get_beginNode()==b && it->get_destinationNode()==d)
                    contained =true;
                it++;
            }
            return contained;
        }
        // Print the nodes containedin the queue.
        void print(){
            std::list<Edge>::iterator i = queue.begin();
            while(i != queue.end()){
                i->print();
                i++;
            }
        }
};

// ===========================================================================================================================
// Prim's Algorithm Class
// An Object of this class contains the mininmun spanning tree of a graph.
// ===========================================================================================================================
class MinSpanTree{
    private:
        std::list<Node> List;
        Graph mstGraph;
        int cost=0;

    public:
        // Constructors
        MinSpanTree();
        // When an object of this class is declared the constructor runs the Prim algorithm to find the minimum spanning tree and its cost
        MinSpanTree(Graph g, Node u){
            Node n;
            Edge e; // Helper edge to put edge in the priority queue
            Edge t; // top edge on the priority queue
            std::list<Edge> queue;
            int min = std::numeric_limits<int>::max(), minNode;
            std::list<Edge> nghbs;                 
            std::list<Edge>::iterator it;

            // Initialize priority Queue
            PriorityQueue q(queue);

            while(mstGraph.get_VG() != g.get_VG()){
                // Insert node u as init node in mst graph.
                mstGraph.add_Node(u.get_name());
                // Fetch the neighboring nodes.
                nghbs = g.get_edgeList(u);
                // Check if there aren't any neighbors.
                if(nghbs.empty()){
                    std::cout << "Node : " << u.get_name() << " has no neigbors" << std::endl;
                    break;
                }
                it = nghbs.begin();
                // Add neigboring edges in the priority queue
                while(it != nghbs.end()){
                    e.set_weight(it->get_weight());
                    e.set_beginNode(it->get_beginNode());
                    e.set_destinationNode(it->get_destinationNode());
                    q.insert(e);
                    it++;
                }
                
                // Get the top edge to put it in mst grap.
                t = q.get_top();
                // Check if edge already exists in the graph
                while(mstGraph.node_exists(t.get_destinationNode()))
                    t = q.get_top();
                // Add node and edge in mst graph
                mstGraph.add_Node(t.get_destinationNode());
                n.set_name(t.get_destinationNode());
                u.set_name(t.get_beginNode());
                mstGraph.add(u,n,t.get_weight());
                // When an new edge is  inputed in the graph it's weight is added to the cost.
                cost += t.get_weight();

                // In the next iteration add in the queue the edges if the new added node.
                u.set_name(t.get_destinationNode());
            }
            
            // Destructor
            //~ShortestPath();//{cout << "ShortestPath destroyed" << endl;}
        }
        // Print mst graph
        void print(){
            mstGraph.print_Graph();
            std::cout << "Cost = " << cost << std::endl;
        }

};

//====================
// Main function
// ==================
int main(){

    // Initial node is 0
    Node x;
    x.set_name(0);

    Graph g("graph.txt");
    //g.print_Graph();

    // Calculate minimum spanning tree in the constructor.
    MinSpanTree mst(g,x);
    // Print tree with its cost.
    mst.print();

    return 0;
}
