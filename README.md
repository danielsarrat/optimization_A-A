# A* algortithm implementation in a map
Repository for the 2nd Optimization delivery

The objective in this project consists in finding an optimal path, within a map of Spain, between two given locations: from Basílica de Santa Maria del Mar (Plaça de Santa Maria) in Barcelona to the Giralda (Calle Mateos Gago) in Sevilla, according to the distance traveled. And the implementation has to be done via an A* algorithm in the programming language C

For the purpose of optimizing the code, as outlined in the assignment, our team developed two different programs. The first program processes the .csv file containing the map node information, generating a binary file that pre-establishes the connections between nodes, hence enabling faster file access (Astar_binaryfile_write.c). The second program reads the binary file and implements the A* algorithm to efficiently compute the solution to the problem (Astar.c).

This two programs can be found in the repository alongside a.csv file with the information of the Andorra's map and a Python script to visualize the optimal path in a Map. This .csv file and the Python Script were provided as a part of an assigment.
