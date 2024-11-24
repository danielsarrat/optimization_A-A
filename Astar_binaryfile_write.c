// - counts the number of nodes in the map
// - allocates memory for the nodes and loads the information of each node.
// - writes the information in a binary file
#include <stdlib.h>
#include <stdio.h>
#include <string.h>
#include <time.h>
#include <stdint.h>

#define CHAR_LENGTH 200

typedef struct {
    unsigned long id; // Node identification
    char *name; // Name of the node (now a pointer)
    double lat, lon;  // Node coordinates
    unsigned short nsucc;  // Number of node successors; i. e. length of successors
    unsigned long *successors; // Array of the successors (now a pointer)
} node;

void ExitError(const char *miss, int errcode) {
fprintf (stderr, "\nERROR: %s.\nStopping...\n\n", miss); exit(errcode);
}

unsigned long searchNode(unsigned long id, node *nodes, unsigned long nnodes);

int main(int argc,char *argv[])
{
    clock_t start_time;
    FILE *mapfile;
    unsigned long nnodes;
    char *line=NULL;
    size_t len;

    start_time = clock();

    char mapname[80];
    strcpy(mapname,"andorra.csv");

    if(argc>1) strcpy(mapname,argv[1]);

    mapfile = fopen(mapname, "r");
    if (mapfile == NULL)
    {
        printf("Error when opening the file\n");
        return 1;
    }
    // count the nodes
    nnodes=0UL;
    while (getline(&line, &len, mapfile) != -1)
    {
        if (strncmp(line, "node", 4) == 0)
        {
            nnodes++;
        }
    }
    printf("Total number of nodes is %ld\n", nnodes);

    rewind(mapfile);
    
    // start_time = clock();
    node *nodes;
    char *tmpline , *field , *ptr;
    unsigned long index=0;

    nodes = (node*) malloc(nnodes*sizeof(node));
    if(nodes==NULL){
        printf("Error while allocating the memory for the nodes\n.");
        return 2;
    }

    while (getline(&line, &len, mapfile) != -1)
    {
        if (strncmp(line, "#", 1) == 0) continue;
        tmpline = line; // make a copy of line to tmpline to keep the pointer of line
        field = strsep(&tmpline, "|");
        if (strcmp(field, "node") == 0)
        {
            field = strsep(&tmpline, "|");
            nodes[index].id = strtoul(field, &ptr, 10);
            field = strsep(&tmpline, "|");

            if (field != NULL && *field != '\0') // Check if the node has a name
            {
                nodes[index].name = (char *)malloc( CHAR_LENGTH * sizeof(char) );
                if (nodes[index].name == NULL){
                    printf("Error while allocating memory for the names.\n");
                    return 2;
                }
                strcpy(nodes[index].name,field);
            }
            else{
                nodes[index].name = (char *)malloc( sizeof(char) );
                if (nodes[index].name == NULL){
                    printf("Error while allocating memory for the names.\n");
                    return 2;
                }
                nodes[index].name[0] = '\0'; // Save an empty string
            }
            
            for (int i = 0; i < 7; i++)
                field = strsep(&tmpline, "|");
            nodes[index].lat = atof(field);
            field = strsep(&tmpline, "|");
            nodes[index].lon = atof(field);

            nodes[index].nsucc = 0; // start with 0 successors

            nodes[index].successors = (unsigned long*)malloc(sizeof(unsigned long)); 
            if (nodes[index].successors == NULL){
                printf("Error while allocating memory for the successors.\n");
                return 2;
            }

            index++;
        }
    }
    printf("Assigned data to %ld nodes\n", index);
    // printf("Elapsed time: %f seconds\n", (float)(clock() - start_time) / CLOCKS_PER_SEC);
    printf("Last node has:\n id=%lu\n GPS=(%lf,%lf)\n",nodes[index-1].id, nodes[index-1].lat, nodes[index-1].lon);
    
    rewind(mapfile);
    
    // start_time = clock();
    int oneway;
    unsigned long nedges = 0, origin, dest, originId, destId;
    while (getline(&line, &len, mapfile) != -1)
    {
        if (strncmp(line, "#", 1) == 0) continue;
        tmpline = line; // make a copy of line to tmpline to keep the pointer of line
        field = strsep(&tmpline, "|");
        if (strcmp(field, "way") == 0)
        {
            for (int i = 0; i < 7; i++) field = strsep(&tmpline, "|"); // skip 7 fields
            if (strcmp(field, "") == 0) oneway = 0; // no oneway
            else if (strcmp(field, "oneway") == 0) oneway = 1;
            else continue; // No correct information
            field = strsep(&tmpline, "|"); // skip 1 field
            field = strsep(&tmpline, "|");
            if (field == NULL) continue;
            originId = strtoul(field, &ptr, 10);
            origin = searchNode(originId,nodes,nnodes);
            while(1)
            {
                field = strsep(&tmpline, "|");
                if (field == NULL) break;
                destId = strtoul(field, &ptr, 10);
                dest = searchNode(destId,nodes,nnodes);
                if((origin == nnodes+1)||(dest == nnodes+1))
                {
                    originId = destId;
                    origin = dest;
                    continue;
                }
                if(origin==dest) continue;
                // Check if the edge did appear in a previous way
                int newdest = 1;
                for(int i=0;i<nodes[origin].nsucc;i++)
                    if(nodes[origin].successors[i]==dest){
                        newdest = 0;
                        break;
                    }
                if(newdest){
                    if (nodes[origin].nsucc > 0){
                        unsigned long *temp_orig;
                        temp_orig = (unsigned long*)realloc(nodes[origin].successors,
                        (nodes[origin].nsucc + 1)*sizeof(unsigned long));

                        if (temp_orig == NULL){
                            printf("Error while reallocating memory for the successors.\n");
                            free(nodes[origin].successors);
                            return 10;
                        }

                        nodes[origin].successors = temp_orig;
                    }

                    nodes[origin].successors[nodes[origin].nsucc]=dest;
                    nodes[origin].nsucc++;
                    nedges++;
                }
                if(!oneway)
                {   
                    // Check if the edge did appear in a previous way
                    int newor = 1;
                    for(int i=0;i<nodes[dest].nsucc;i++)
                        if(nodes[dest].successors[i]==origin){
                            newor = 0;
                            break;
                        }
                    if(newor){
                        if (nodes[dest].nsucc > 0){
                        unsigned long *temp_dest;
                        temp_dest = (unsigned long*)realloc(nodes[dest].successors,
                        (nodes[dest].nsucc + 1)*sizeof(unsigned long));

                        if (temp_dest == NULL){
                            printf("Error while reallocating memory for the successors.\n");
                            free(nodes[dest].successors);
                            return 10;
                        }

                        nodes[dest].successors = temp_dest;
                    }
                        nodes[dest].successors[nodes[dest].nsucc]=origin;
                        nodes[dest].nsucc++;
                        nedges++;
                    }
                }
                originId = destId;
                origin = dest;
            }
        }
    }
    
    fclose(mapfile);
    printf("Assigned %ld edges\n", nedges);

    printf("\nWriting the binary file...\n");

    FILE *binmapfile;
    char binmapname[80];
    strcpy(binmapname,mapname);
    strcat(binmapname,".bin");

    binmapfile = fopen(binmapname,"wb");

    /* Global data −−− header */
    if( fwrite(&nnodes, sizeof(unsigned long), 1, binmapfile) +
    fwrite(&nedges, sizeof(unsigned long), 1, binmapfile) != 2 )
    ExitError("when initializing the output binary data file", 32);

    /* Writing all nodes */
    if( fwrite(nodes, sizeof(node), nnodes, binmapfile) != nnodes )
    ExitError("when writing nodes to the output binary data file", 32);

    /* Writing sucessors in blocks */
    for(int i=0; i < nnodes; i++) if(nodes[i].nsucc) {
        if( fwrite(nodes[i].successors, sizeof(unsigned long), nodes[i].nsucc, binmapfile) !=
        nodes[i].nsucc ){
            
            ExitError("when writing edges to the output binary data file", 32);
        }
        free(nodes[i].successors);
        free(nodes[i].name);
    }

    free(nodes);
    free(line);

    fclose(binmapfile);

    printf("Binary file written succesfully.\n\n");

    printf("Elapsed time: %f seconds\n", (float)(clock() - start_time) / CLOCKS_PER_SEC);

    return 0;

}

unsigned long searchNode(unsigned long id, node *nodes, unsigned long nnodes)
{
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
