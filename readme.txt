=== Using this code ===

To change the input file, change the string "inputFileName"
in the main method of maxflow.py. Then run the file with

> python3 maxflow.py

The main method will execute and you will get:
1) a graph of the network from that input file
2) the flow output by BFS
3) the flow output by Dijkstra
4) the comparison between BFS and Dijkstra


=== Assumptions ===

The input file will contain at least two nodes 1 and 2.
1) 1 will be the source.
2) 2 will be the sink.
3) there is always a path/flow from 1 to 2.
