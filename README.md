# Data-Structures-and-Algorithms
- A two-phase project for course 'Data structures and Algorithms" at the University of Tampere
- Folder prg2 include teacher assignments called "prg1-game-of-taxes-en" and "prg2-connecting-towns-en"
- The UI code was given to students and the main purpose was to implement all the algorithms. datastructures.cc is fully implemented by me and can maybe be worth checking. 

## Usage
1. Can be used with Qt Creator or with terminal as showed below
2. Install Qt for MAC with $brew install qt
3. Clone this project
4. $cd ~/path_to/prg2
5. $qmake prg2.pro 
6. $make
7. head to the prg2 folder and open prg2 application

## Example use cases 
- Program includes help command and information of usage can be also found from pdf "prg2-connecting-towns-en"
- Prg2 includes some .txt files for easy usage. With these user can read some test data (towns and vassals) for the program
- For example by reading the towns-data.txt file the program looks like below

![Use cases](/pics/Example.png)

- There are some loops and the shortest route from Helsinki to Utsjoki can be found with command shortest_route. This is implemented with A*-algorithm

![Use cases](/pics/Dijkstra.png)

- Also, if we want for example to trim the road network from loops, it can be done with trim_road_network. So called spanning tree problem can be solved with Prim's algorithm.

![Use cases](/pics/Prim.png)
