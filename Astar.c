#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <float.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define EARTH_RADIUS 6371.0
#define M_PI 3.14159265358979323846

typedef char bool; 
enum {false, true};

typedef struct{ //Structure that stores all the nodes and its data
    unsigned long id; 
    char *name;
    double lat, lon;
    unsigned short nsucc;
    unsigned long *successors;
} node; 

typedef struct { //Structure that we will use to store the data to build the shortest path
    float g; 
    unsigned parent; 
} AStarPath;

typedef struct QueueElementstruct { 
    unsigned v; 
    struct QueueElementstruct *seg; 
} QueueElement;

typedef QueueElement * PriorityQueue;

typedef struct { //Structure to store the estimated cost of each node and in which list it is
    float f; 
    bool IsOpen; 
} AStarControlData;


void ExitError(const char *miss, int errcode);
unsigned searchNode(unsigned long id, node *nodes, unsigned long nnodes);
float heuristic(node *Graph, unsigned node1, unsigned node2);
bool AStar(node *Graph, AStarPath *Closed, unsigned GrOrder, unsigned node_start, unsigned node_goal, unsigned long *expanded_nodes);

int main(int argc, char *argv[]) {
    
    clock_t start_time;
    unsigned long nnodes;
    unsigned long ntotnsucc = 0UL;                         
    unsigned long *allsuccessors = NULL;

    start_time = clock();

    char binmapname[86];

    if (argc == 3){
        strcpy(binmapname,argv[1]);
    }

    else if (argc != 3)
    {
        if (argc >= 2){
            printf("Wrong number of arguments! input should look like this:\n./program_name <binary_file_name> <node1> <node2> \nA demo will be computed if you enter a valid map file in the 1st argument\n");
            strcpy(binmapname,argv[1]);
        }
        else{
            printf("Wrong number of arguments! input should look like this:\n./program_name <binary_file_name> <node1> <node2> \nWe will try to compute a demo using Andorra's map \n");
            strcpy(binmapname,"andorra.csv.bin");
        }        
    }    
    
    FILE *binmapfile;


    if ((binmapfile = fopen(binmapname,"rb")) == NULL){
        printf("Invalid map file name introduced in the first argument \nStopping the program\n");
        return 1;
    }
    
    node *nodes;

    //Read binary file

    /* Global data −−− header */
    if(fread(&nnodes, sizeof(unsigned long), 1, binmapfile) + fread(&ntotnsucc, sizeof(unsigned long), 1, binmapfile) != 2 )
        ExitError("when reading the header of the binary data file", 12);

    /* getting memory for all data */
    if((nodes = (node *) malloc(nnodes*sizeof(node))) == NULL)
        ExitError("when allocating memory for the nodes vector", 13);
    if((allsuccessors = (unsigned long *) malloc(ntotnsucc*sizeof(unsigned long))) == NULL)
        ExitError("when allocating memory for the edges vector", 15);

    /* Reading all data from file */
    if(fread(nodes, sizeof(node), nnodes, binmapfile) != nnodes )
        ExitError("when reading nodes from the binary data file", 17);
    if(fread(allsuccessors, sizeof(unsigned long), ntotnsucc, binmapfile) != ntotnsucc)
        ExitError("when reading sucessors from the binary data file", 18);
    
    /* Setting pointers to successors */
    for(size_t i=0; i <nnodes; i++) if(nodes[i].nsucc) {
        nodes[i].successors = allsuccessors; allsuccessors += nodes[i].nsucc;}

    AStarPath* Closed = (AStarPath*)malloc(nnodes * sizeof(AStarPath));
    if (Closed == NULL) {
        perror("Failed to allocate memory for Closed");
        exit(EXIT_FAILURE);
    }
    
    // Convert command line arguments to unsigned longs
    unsigned long start_id = 0;
    unsigned long goal_id = 0;

    if (argc == 3) {
        start_id = strtoul(argv[1], NULL, 10);
        goal_id = strtoul(argv[2], NULL, 10);
    } else if (argc >= 1 && argc != 3) {
        const char *binmapname = argv[1]; // Assuming this is passed as argv[0]

        if (strcmp(binmapname, "andorra.csv.bin") == 0) {
            start_id = 625025;
            goal_id = 625051;
        } else if (strcmp(binmapname, "catalunya.csv.bin") == 0) {
            start_id = 240949599;
            goal_id = 1351114961;
        } else if (strcmp(binmapname, "spain.csv.bin") == 0) {
            start_id = 240949599;
            goal_id = 195977239;
        } else {
            printf("No demo available for the first introduced argument\n");
            return -1;
        }
    } else {
        printf("Invalid number of arguments\n");
        return -1;
    }
    
    unsigned start = searchNode(start_id, nodes, nnodes);
    unsigned target = searchNode(goal_id, nodes, nnodes);
    
    // Check if the nodes exist in the graph
    if (start >= nnodes || target >= nnodes) {
        ExitError("invalid node ID(s)", 3);
    }

    //Astar algorithm execution
    unsigned long expanded_nodes;
    bool r = AStar(nodes, Closed, nnodes, start, target, &expanded_nodes);

    if(r == -1) ExitError("in allocating memory for the OPEN list in AStar", 21);
    else if(!r) ExitError("no solution found in AStar", 7);


    // Construction of the optimal path from the Astar algorithm results
    register unsigned v = target, pv = Closed[v].parent, ppv; 
    
    Closed[target].parent = UINT_MAX;
    
    while(v != start) { 
        ppv=Closed[pv].parent; 
        Closed[pv].parent=v; 
        v=pv; 
        pv=ppv; 
    }
    
    //Write in a txt file
    FILE *file;

    // Open the file in write mode
    file = fopen("path.txt", "w");
    if (file == NULL) {
        perror("Error opening file");
        return EXIT_FAILURE;
    }
    // Write formatted text to the file
    fprintf(file, "# We have expanded %lu nodes\n", expanded_nodes);
    fprintf(file, "# Distance from %lu to %lu: %f km.\n", start_id, goal_id,Closed[target].g);
    fprintf(file, "# Optimal path:\n");

    for(v = Closed[start].parent ; v !=UINT_MAX ; v = Closed[v].parent) //save the 
        fprintf(file, "Id = %lu | %f | %f | Dist = %f\n", nodes[v].id, nodes[v].lat, nodes[v].lon, Closed[v].g );
    // Close the file
    fclose(file);
    
    printf("Total elapsed time: %f seconds\n", (float)(clock() - start_time) / CLOCKS_PER_SEC); //Complete program execution time
    printf("The optimal path is %f km long.\n",Closed[target].g);
    printf("The list of nodes compoisng the optimal path can be read in the generated path.txt\n");
    return 0; 
}

void ExitError(const char *miss, int errcode) {
    fprintf (stderr, "\nERROR: %s.\nStopping...\n\n", miss); 
    exit(errcode);
}

float heuristic(node *Graph, unsigned node1, unsigned node2){ //We will use the haversine both as an heuristic and to compute the distance between nodes

    float dlat = (Graph[node2].lat - Graph[node1].lat)* M_PI / 180.0;
    float dlon = (Graph[node2].lon - Graph[node1].lon)* M_PI / 180.0;

    float lat1 = (Graph[node1].lat)* M_PI / 180.0;
    float lat2 = (Graph[node2].lat)* M_PI / 180.0;

    float a = sin(dlat / 2.0) * sin(dlat / 2.0) + cos(lat1) * cos(lat2) * sin(dlon / 2.0) * sin(dlon / 2.0);

    float c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

    return EARTH_RADIUS*c;
}

unsigned searchNode(unsigned long id, node *nodes, unsigned long nnodes){
    // we know that the nodes where numrically ordered by id, so we can do a binary search.
    unsigned long l = 0, r = nnodes - 1, m;
    while (l <= r)
    {
        m = l + (r - l) / 2;
        if (nodes[m].id == id) return m;
        if (nodes[m].id < id)
            l = m + 1;
        else
            r = m - 1;
    }

    // id not found, we return nnodes+1
    return nnodes+1;
}

bool IsEmpty(PriorityQueue Pq){
    return ((bool) (Pq == NULL));
}

unsigned extract_min(PriorityQueue *Pq){
    PriorityQueue first = *Pq;
    unsigned v = first->v;
    *Pq = (*Pq)->seg;
    free(first);
    return v; 
}

bool add_with_priority(unsigned v, PriorityQueue *Pq, AStarControlData * Q){

    register QueueElement * q;
    
    QueueElement *aux = (QueueElement *) malloc(sizeof(QueueElement));

    if(aux == NULL) return false;

    aux->v = v;
    
    float costv = Q[v].f;
    Q[v].IsOpen = true;

    if( *Pq == NULL || !(costv > Q[(*Pq)->v].f) ) {
        aux->seg = *Pq; 
        *Pq = aux;
        return true;
    }
    
    for(q = *Pq; q->seg && Q[q->seg->v].f < costv; q = q->seg ) ;
    aux->seg = q->seg; 
    q->seg = aux;
    return true;
}

void requeue_with_priority(unsigned v, PriorityQueue *Pq, AStarControlData * Q){
    
    register QueueElement * prepv;
    if((*Pq)->v == v) return;
    for(prepv = *Pq; prepv->seg->v != v; prepv = prepv->seg);
    QueueElement * pv = prepv->seg;
    prepv->seg = pv->seg;
    free(pv);
    add_with_priority(v, Pq, Q); 
}

bool AStar(node *Graph, AStarPath *Closed, unsigned GrOrder, unsigned node_start, unsigned node_goal, unsigned long *expanded_nodes){ 
    register unsigned i;
    
    PriorityQueue Open = NULL;
    AStarControlData *Q;
    *expanded_nodes = 0;

    if((Q = (AStarControlData *) malloc(GrOrder*sizeof(AStarControlData))) == NULL){ 
        ExitError("when allocating memory for the AStar Control Data vector", 73);
    }
    //set by default all the cost to infinite and all the values not in the OpenList
    for(i=0; i < GrOrder; i++) {
        Closed[i].g = FLT_MAX;
        Q[i].IsOpen = false; 
    } 
    
    //The first element we will inspect have 0 cost to reach but no parent. We put that element in the OpenList
    Closed[node_start].g = 0.0; 
    Closed[node_start].parent = UINT_MAX; 
    Q[node_start].f = heuristic(Graph, node_start, node_goal); 
    if(!add_with_priority(node_start, &Open, Q))
        return -1;

    //Start of the Astar loop
    while(!IsEmpty(Open)){ 

        (*expanded_nodes)++;
        unsigned node_curr;
        
        //If the current node is the target we've finished
        if((node_curr = extract_min(&Open)) == node_goal) {
            free(Q); 
            return true;
        }

        //If not, we explore each neigbhour of the current node
        for(i=0; i < Graph[node_curr].nsucc ; i++){

            unsigned node_succ = Graph[node_curr].successors[i];
            float dist = heuristic(Graph, node_curr, node_succ);
            float g_curr_node_succ = Closed[node_curr].g + dist;

            //If the new computed cost is less than the previous cost
            if( g_curr_node_succ < Closed[node_succ].g ){
                Closed[node_succ].parent = node_curr;

                //We recompute the total distance of the neighbour
                Q[node_succ].f = g_curr_node_succ + ((Closed[node_succ].g == FLT_MAX) ? 
                //If it hasn't been explored before as the agregated cost + heuristic from the neighbour to the target
                heuristic(Graph, node_succ, node_goal) :
                //If it has been explored before as the previously computed heuristic minus the difference of the new aggregated cost
                (Q[node_succ].f-Closed[node_succ].g) );
                
                //The parent will be the new node from whihc arriving is cheaper
                Closed[node_succ].g = g_curr_node_succ;
                
                //If the node hadn't been explored before we add it to the cue with priority (regarding its f)
                if(!Q[node_succ].IsOpen) { 
                    if(!add_with_priority(node_succ, &Open, Q)) return -1; 
                }

                //If the node had been explored before we re-order the cue with priority (regarding its f)
                else requeue_with_priority(node_succ, &Open, Q);
            }
        }
    //Now that we have expanded the node we remove it from the open list and count it as expanded
    Q[node_curr].IsOpen = false;

    }
   return false;
}