#include <stdio.h>
#include <stdlib.h>
#include <limits.h>
#include <float.h>
#include <string.h>
#include <math.h>
#include <time.h>

#define M_PI 3.14159265358979323846
#define EARTH_RADIUS 6371.0
#define MAXSUCC 9

typedef char bool; 
enum {false, true};


typedef struct{ 
    unsigned long id;
    char name[200];
    double lat, lon;
    unsigned short nsucc;
    unsigned long *successors;
} node; // 

typedef struct { 
    float g; 
    unsigned parent; 
} AStarPath;

typedef struct QueueElementstruct { 
    unsigned v; 
    struct QueueElementstruct *seg; 
} QueueElement;

typedef QueueElement * PriorityQueue;

typedef struct { 
    float f; 
    bool IsOpen; 
} AStarControlData;


void ExitError(const char *miss, int errcode);
unsigned searchNode(unsigned long id, node *nodes, unsigned long nnodes);
float heuristic(node *Graph, unsigned vertex, unsigned goal);
float eucl_dist(node *Graph, unsigned vertex_1, unsigned vertex_2);
float haversine_dist(node *Graph, unsigned vertex_1, unsigned vertex_2);
bool AStar(node *Graph, AStarPath *PathData, unsigned GrOrder, unsigned node_start, unsigned node_goal);

int main(int argc, char *argv[]) {

    // Check if we have the right number of arguments
    clock_t start_time;
    unsigned long nnodes;
    unsigned long ntotnsucc = 0UL;                          // Initialize the total number of successors
    unsigned long *allsuccessors = NULL;

    start_time = clock();

    char binmapname[80];
    strcpy(binmapname,"andorra.csv.bin");

    if(argc>1) strcpy(binmapname,argv[1]);

    FILE *binmapfile;

    binmapfile = fopen(binmapname,"rb");
    node * nodes;
    
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

    printf("First node ID: %lu \n",nodes[0].id);
    printf("First node lat: %f \n",nodes[0].lat);
    printf("First node lom: %f \n",nodes[0].lon);
    printf("First node nscu: %hu \n",nodes[0].nsucc);

    printf("Total number of nodes is %ld\n", nnodes);
    printf("Elapsed time: %f seconds\n", (float)(clock() - start_time) / CLOCKS_PER_SEC);

    AStarPath PathData[nnodes];
    printf("Hola\n");
    
    // Convert command line arguments to unsigned longs
    unsigned long start_id = strtoul(argv[2], NULL, 10);
    unsigned long goal_id = strtoul(argv[3], NULL, 10);
    
    unsigned node_start = searchNode(start_id, nodes, nnodes);
    unsigned node_goal = searchNode(goal_id, nodes, nnodes);

    // Check if the nodes exist in the graph
    if (node_start >= nnodes || node_goal >= nnodes) {
        ExitError("invalid node ID(s)", 3);
    }

    bool r = AStar(nodes, PathData, nnodes, node_start, node_goal);
    
    if(r == -1) ExitError("in allocating memory for the OPEN list in AStar", 21);
    else if(!r) ExitError("no solution found in AStar", 7);

    register unsigned v=node_goal, pv=PathData[v].parent, ppv; 
    
    PathData[node_goal].parent = UINT_MAX;
    
    while(v != node_start) { 
        ppv=PathData[pv].parent; 
        PathData[pv].parent=v; 
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
    fprintf(file, "# We have expanded payaso nodes\n");
    fprintf(file, "# Distance from %lu to %lu: %f metres.\n", start_id, goal_id,PathData[node_goal].g);
    fprintf(file, "# Optimal path:\n");

    for(v=PathData[node_start].parent ; v !=UINT_MAX ; v=PathData[v].parent)
        fprintf(file, "Id = %lu | %f | %f | Dist = %f\n", nodes[v].id, nodes[v].lat, nodes[v].lon, PathData[v].g );
    
    return 0;
    // Close the file
    fclose(file);

    printf("Optimal path found:\nNode ID | Distance\n----------|---------\n");
    printf(" %lu (%3.3u) | Source\n", nodes[node_start].id, node_start);
    
    for(v=PathData[node_start].parent ; v !=UINT_MAX ; v=PathData[v].parent)
        printf(" %lu (%3.3u) | %7.3f\n", nodes[v].id, v, PathData[v].g);
    
    return 0; 
}

float to_radians(float degrees) {
    return degrees * M_PI / 180.0;
}

void ExitError(const char *miss, int errcode) {
    fprintf (stderr, "\nERROR: %s.\nStopping...\n\n", miss); 
    exit(errcode);
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

float heuristic(node *Graph, unsigned vertex, unsigned goal){
    // register unsigned short i;
    if(vertex == goal) return 0.0;

    float dist = haversine_dist(Graph, vertex, goal);
    
    return dist; 
}

float eucl_dist(node *Graph, unsigned vertex_1, unsigned vertex_2){
    
    float avgLat = to_radians((Graph[vertex_2].lat + Graph[vertex_1].lat)/2);

    float dx = to_radians(Graph[vertex_2].lon - Graph[vertex_1].lon) * cos(avgLat);
    float dy = to_radians(Graph[vertex_2].lat - Graph[vertex_1].lat);

    float dist = sqrt(dx*dx + dy*dy) * EARTH_RADIUS;
    return dist; 
}

float haversine_dist(node *Graph, unsigned vertex_1, unsigned vertex_2){

    float dLat = to_radians(Graph[vertex_2].lat - Graph[vertex_1].lat);
    float dLon = to_radians(Graph[vertex_2].lon - Graph[vertex_1].lon);

    float lat1 = to_radians(Graph[vertex_1].lat);
    float lat2 = to_radians(Graph[vertex_2].lat);

    float a = sin(dLat / 2.0) * sin(dLat / 2.0) +
               cos(lat1) * cos(lat2) * sin(dLon / 2.0) * sin(dLon / 2.0);

    float c = 2.0 * atan2(sqrt(a), sqrt(1.0 - a));

    return EARTH_RADIUS * c;
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

bool AStar(node *Graph, AStarPath *PathData, unsigned GrOrder, unsigned node_start, unsigned node_goal){ 
    register unsigned i;
    
    PriorityQueue Open = NULL;
    AStarControlData *Q;
    
    if((Q = (AStarControlData *) malloc(GrOrder*sizeof(AStarControlData))) == NULL){ 
        ExitError("when allocating memory for the AStar Control Data vector", 73);
    }

    for(i=0; i < GrOrder; i++) {

        PathData[i].g = FLT_MAX;
        Q[i].IsOpen = false; 

    } // inicializando g maximo
    
    PathData[node_start].g = 0.0; // inicio g a cero
    PathData[node_start].parent = UINT_MAX; // nodo de inicio esta huerfano
    Q[node_start].f = heuristic(Graph, node_start, node_goal); // guarda en astarcontroldata el valor de la funcion heuristica (minimo del grafo)
    
    if(!add_with_priority(node_start, &Open, Q)) return -1;
    while(!IsEmpty(Open)){ 
        unsigned node_curr;
        if((node_curr = extract_min(&Open)) == node_goal) { free(Q); return true; }
        
        for(i=0; i < Graph[node_curr].nsucc ; i++){
            unsigned node_succ = Graph[node_curr].successors[i];
            float dist = haversine_dist(Graph, node_curr, node_succ);
            float g_curr_node_succ = PathData[node_curr].g + dist;
            
            if( g_curr_node_succ < PathData[node_succ].g ){
                PathData[node_succ].parent = node_curr;
                Q[node_succ].f = g_curr_node_succ + ((PathData[node_succ].g == FLT_MAX) ? 
                    heuristic(Graph, node_succ, node_goal) : (Q[node_succ].f-PathData[node_succ].g) );
                PathData[node_succ].g = g_curr_node_succ;
                if(!Q[node_succ].IsOpen) { if(!add_with_priority(node_succ, &Open, Q)) return -1; }
                else requeue_with_priority(node_succ, &Open, Q);
            }
        }
    Q[node_curr].IsOpen = false;
    } 
    return false;
}