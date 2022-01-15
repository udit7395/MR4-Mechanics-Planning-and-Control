Folder Structure
code    ->  prm.py

results ->  edges.csv
        ->  nodes.csv
        ->  obstacles.csv
        ->  path.csv

screenshot_results ->  edges.csv
                   ->  nodes.csv
                   ->  obstacles.csv
                   ->  path.csv

output.png

The screenshot_results folder contains the output that used to produce 'output.png' generated after executing the code incase you want to verify it.

1.I have choosen to program a PRM. To help debug and vizualize issues while coding I have used the matplot library in python.
The boolean variable 'PLOTTING' when set to false will disable all plots.

2.I generate random samples using numpy's uniform sample function along the X and Y direction.
3.I add this point to the node list if and only if it is outside the obstacles.
4.I increase the size of the obstacles abit while checking so as to into consideration the radius of the 'Kilo' bot to avoid collision with the obstacles.

5.The class Node hosts all the information regarding its neighbours, optmistic cost, past cost etc.

6.To find the neighbours to a given node 'A' I find the euclidean distance between the given node 'A' and other remaining nodes in the list.
I sort this list on the basis of euclidean distance and pick 5 closest nodes.

7.With the above step completed I have generated the tree and now using A* solve for the shortest path.

8.Finally I write the output to the respective files in results folder.

Incase anything is wrong or output is not as expected, please let me know in the review.
Thank you.