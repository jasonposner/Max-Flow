Prof's Assumptions:

Assumption:
The input file will contain at least two nodes 1 and 2.
1) 1 will be the source.
2) 2 will be the sink.
3) there is always a path/flow from 1 to 2.


Current Problems:

1. When running edge_list3, the program will return the incorrect max flow.
It should be 6 but it returns 8.


[Fixed]
When running edge_list3 (an edge case where there are two nodes), the
program crashes
