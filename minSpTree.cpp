/*   Create a random undirected graph and find the minimum spanning tree.
 *   Thomas Pliakis
 *   January 11, 2023
 */

#include <fstream>
#include <algorithm>
#include <iostream>
#include <unistd.h>
#include <limits>
#include <random>
#include <queue>
#include <vector>
#include <iterator>
#include <list>
#include <ctime>

// ============================================================================
// Definitions
// ============================================================================
const int N=10;
const int NUMOFNODES =50;
const int maxNumOfEdges = (NUMOFNODES *(NUMOFNODES -1))/2;


// ============================================================================
// Random Engine for correct densities
// ============================================================================
std::random_device rd;
std::uniform_int_distribution<int> distribution(1, 100);
std::mt19937 engine(rd()); // Mersenne twister MT19937

//=============================================================================
// Edge Class
// The objects of this class are the edges of our graph 
// that keep the destination of the edge and its weight.
// ============================================================================
class Edge{
    private:
        int destinationNode;
        int weight;
    public:
        Edge(int d, int w){destinationNode = d; weight =w;}
        Edge():destinationNode(0),weight(0){}
        //~Edge();

        // Methods
        void print(){
            std::cout << "     ";
            std::cout << "DestinaitonNode : " << destinationNode << "\tWeight: " << weight << std::endl;
        }
        //  Set and Get methods
        void set_destinationNode(int d){destinationNode = d;}
        int get_destinationNode(){return destinationNode;}
        void set_weight(int w){weight = w;}
        int get_weight(){return weight;}
};

// ==================================================================================================
// Node Class
// The objects of this class are the Nodes of our graph that keep the the id(name) of the node,
// the value which is the path length from the start to this node and
// a list with  edges to the other nodes that this node is connected to.
// =================================================================================================
class Node{
    private:
        int name;
        int value;
        std::list<Edge> edgeList;
    public:
        // Contructors
        Node(int name , std::list<Edge> edgeList){this->name = name;value = std::numeric_limits<int>::max();this->edgeList=edgeList;}
        Node():name(0),value(std::numeric_limits<int>::max()),edgeList(){} // Do we have to add edgelist or it doesnt matter?
                                                                      //  ~Node(){cout << "Node "<< name << " destroyed"<< endl;}

                                                                      //Methods
        void print(){
            std::cout << "Node no: " << name << " value: " << value << std::endl;

            std::list<Edge>::iterator it=edgeList.begin();
            while(it!=edgeList.end()){
                it->print();
                it++;
            }
        }
        // Set and Get
        void set_name(int n){name = n;}
        int get_name(){return name;};
        void set_value(int p){value = p;};
        int get_value(){return value;};
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
        // Adding a edge
        void add_edge(Node y,int w){
            Edge e(y.get_name(),w); //declare a variable of type edge
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
        Graph(int density, int range){
            Node n,x,y;

            int value;

            // Create a graph of size NUMOFNODES
            for(int i=0; i<NUMOFNODES; i++){
                add_Node(i);
            }
            
            // Initialize the edges with density 20%
            //maxNumOfEdges=1225 loops with NUMOFNODES=50
            for(int i=0; i<NUMOFNODES; i++){
                for(int j=i+1; j<NUMOFNODES; j++){ 
                    value=distribution(engine);
                    // value must be less than 20 to have 20% density
                    if(value <= density){ 
                        x.set_name(i);
                        y.set_name(j);
                        add(x,y,range);       // insert edge between nodes x,y
                    }
                }
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
        bool add(Node x, Node y, int range){
            if(nodes[x.get_name()].check_edge(y.get_name()) || x.get_name()==y.get_name()){
                return 1;
            }else{
                int w = rand()%range+1;
                nodes[x.get_name()].add_edge(y,w);
                nodes[y.get_name()].add_edge(x,w);
                EG++;
                return 1;
            }
            return 0;
        }
        // Set and Get path lengths
        int  get_node_value(Node n){
            int v=0;
            std::vector<Node>::iterator ve;
            for(ve=nodes.begin(); ve!=nodes.end(); ve++){
                if(ve->get_name() == n.get_name()){
                    return ve->get_value();
                }
            }
            return v;
        }
        void set_node_value(Node n,int v){
            std::vector<Node>::iterator ve;
            for(ve=nodes.begin(); ve!=nodes.end(); ve++){
                if(ve->get_name() == n.get_name()){
                    ve->set_value(v);
                }
            }
        }
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
// The class of our Priority Queue which contains a  vector of nodes.
// The node at the top of the queue always has the minimum value (path length) and so its the next to picked.
// ====================================================================================================================
class PriorityQueue {
    private:
        std::vector<Node> queueOfNodes;

    public:
        // COnstructor
        PriorityQueue();
        PriorityQueue(std::vector<Node> queueOfNodes){this->queueOfNodes = queueOfNodes;}
        //~PriorityQueue(){cout << "PQ destroyed" << endl;}

        //Methods
        // Set and Get
        Node get_top(){
            std::vector<Node>::iterator it = queueOfNodes.begin() ;
            Node n = queueOfNodes[0];
            queueOfNodes.erase(it);
            return n;
        }
        int get_size(){return queueOfNodes.size();}
        int  get_value(Node n){
            int v=0;
            std::vector<Node>::iterator it;
            for(it = queueOfNodes.begin(); it < queueOfNodes.end(); it++){
                if( n.get_name() == it->get_name()){
                    return  it->get_value();
                }
            }
            return v;
        }
        // Insert a node to the queue based on its value, the lowest the value the higher in the queue.
        void insert(Node queue_element){
            std::vector<Node>::iterator it;
            bool inserted = false;
            for(it = queueOfNodes.begin(); it<queueOfNodes.end(); it++){
                if( queue_element.get_value() < it->get_value()){
                    queueOfNodes.insert(it,queue_element);
                    inserted = true;
                    break;
                }
            }
            if(!inserted)
                queueOfNodes.push_back(queue_element);
        }
        // Update the value(path length) of a node
        void updateValue(Node n, int newValue){
            n.set_value(newValue);
            // When a valua of a node is changed we need to change its position in the queue
            chgPrioirity(n);
        }
        // To change a nodes position in the queue i erase it from its current posiion and i insert it again with the new value.
        void chgPrioirity(Node n){
            std::vector<Node>::iterator it;
            for(it = queueOfNodes.begin(); it < queueOfNodes.end(); it++){
                if( n.get_name() == it->get_name()){
                    queueOfNodes.erase(it);
                }
            }
            insert(n);
        }
        // Check if a node is contained in the queue.
        bool contains(int name){
            std::vector<Node>::iterator it;
            bool contained = false;
            for(it = queueOfNodes.begin(); it < queueOfNodes.end(); it++){
                if( name == it->get_name()){
                    contained = true;
                }
            }
            return contained;
        }
        // Print the nodes containedin the queue.
        void print(){
            std::vector<Node>::iterator i = queueOfNodes.begin();
            while(i != queueOfNodes.end()){
                i->print();
                i++;
            }
        }
};

// ===========================================================================================================================
// Dijkstra Algorithm Class
// An Object of this class contains a graph, in which each node has its path length (value) calculated in the constructor.
// ===========================================================================================================================
class ShortestPath{
    private:
        std::list<Node> List;
        Graph DikjstraGraph;

    public:
        // Constructors
        ShortestPath();
        // When an object of this class is declared the constructor runs the dijkstra algorithm
        ShortestPath(Graph g, Node u){
            std::vector<Node> queueOfNodes;
            PriorityQueue PQ(queueOfNodes);   // Init the Priority queue
            Node n,m;                         // Helper nodes
            std::list<Edge> nghbs;                 
            std::vector<int> sp;
            int newValue;
            std::list<Edge>::iterator li;

            // Set node u ad init node
            g.set_node_value(u,0);

            // Initialize priority Queue
            for(int i=0; i<g.get_VG(); i++){
                n = g.get_Node(i);
                PQ.insert(n);
            }
            //g.print_Graph();

            while(PQ.get_size()){
                m = PQ.get_top();
                DikjstraGraph.add_node(m);

                // Update the distances to neighbors
                nghbs = g.get_edgeList(m);
                for(li=nghbs.begin(); li!=nghbs.end(); li++){
                    if(PQ.contains(li->get_destinationNode())){
                        n = g.get_Node(li->get_destinationNode());
                        newValue = g.get_weight(n,m) + DikjstraGraph.get_node_value(m);
                        if(PQ.get_value(n) > newValue){
                            PQ.updateValue(n,newValue);
                        }
                    }
                }
            }
            //DikjstraGraph.print_Graph();
        }
        // Destructor
        //~ShortestPath();//{cout << "ShortestPath destroyed" << endl;}

        // Average path calculation
        float calc_avg_path_length(){
            float avg=0.0;
            float count=0.0;
            int dist=0;
            std::vector<Node> nodes = DikjstraGraph.getVecOfNodes();
            std::vector<Node>::iterator it;
            for(it = nodes.begin(); it < nodes.end(); it++){
                dist = it->get_value();
                if(dist !=std::numeric_limits<int>::max() && dist >= 0){
                    avg += dist;
                }
            }
            return  avg/NUMOFNODES;
        }
};

void getw(std::string& t, std::ifstream& in){
    in >> t;
}

//====================
// Main function
// ==================
int main(){

    // Initialize variables
    Graph g(20,10);
    Node x;

    // random engine
    srand(time(0));

    //g.print_Graph();
    x.set_name(0); // Start node for the Dijkstra algorithm is 0
    ShortestPath sh(g,x);
    std::cout << "Average path length with density = 20% : " << sh.calc_avg_path_length() << std::endl;

    Graph g2(40,10);
    //g2.print_Graph();
    x.set_name(0);
    ShortestPath sh2(g2,x);
    std::cout << "Average path length with density = 40% : " << sh2.calc_avg_path_length() << std::endl;

    return 0;
}
