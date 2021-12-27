// Datastructures.hh

#ifndef DATASTRUCTURES_HH
#define DATASTRUCTURES_HH

#include <string>
#include <vector>
#include <tuple>
#include <utility>
#include <limits>
#include <functional>
#include <exception>
#include <unordered_map>
#include <map>
#include <algorithm>
#include <math.h>
#include <unordered_set>
#include <set>
#include <deque>
#include <queue>
#include <stack>
#include <limits>

// Types for IDs
using TownID = std::string;
using Name = std::string;
using Colour = std::string;

// Return values for cases where required thing was not found
TownID const NO_TOWNID = "----------";

// Return value for cases where integer values were not found
int const NO_VALUE = std::numeric_limits<int>::min();

// Return value for cases where name values were not found
Name const NO_NAME = "!!NO_NAME!!";

Colour const WHITE = "WHITE";
Colour const GRAY = "GRAY";
Colour const BLACK = "BLACK";

// Type for a coordinate (x, y)
struct Coord
{
    int x = NO_VALUE;
    int y = NO_VALUE;
};


// Example: Defining == and hash function for Coord so that it can be used
// as key for std::unordered_map/set, if needed
inline bool operator==(Coord c1, Coord c2) { return c1.x == c2.x && c1.y == c2.y; }
inline bool operator!=(Coord c1, Coord c2) { return !(c1==c2); } // Not strictly necessary

struct CoordHash
{
    std::size_t operator()(Coord xy) const
    {
        auto hasher = std::hash<int>();
        auto xhash = hasher(xy.x);
        auto yhash = hasher(xy.y);
        // Combine hash values (magic!)
        return xhash ^ (yhash + 0x9e3779b9 + (xhash << 6) + (xhash >> 2));
    }
};

// Example: Defining < for Coord so that it can be used
// as key for std::map/set
inline bool operator<(Coord c1, Coord c2)
{
    if (c1.y < c2.y) { return true; }
    else if (c2.y < c1.y) { return false; }
    else { return c1.x < c2.x; }
}


Coord const ORIGO = {0,0};

// Return value for cases where coordinates were not found
Coord const NO_COORD = {NO_VALUE, NO_VALUE};

// Type for a distance (in metres)
using Distance = int;

// Return value for cases where Distance is unknown
Distance const NO_DISTANCE = NO_VALUE;

// This exception class is there just so that the user interface can notify
// about operations which are not (yet) implemented
class NotImplemented : public std::exception
{
public:
    NotImplemented() : msg_{} {}
    explicit NotImplemented(std::string const& msg) : msg_{msg + " not implemented"} {}

    virtual const char* what() const noexcept override
    {
        return msg_.c_str();
    }
private:
    std::string msg_;
};


// Implementeation for single town as a struct
struct Town{
    //prg1
    TownID id_;
    std::string name_;
    Coord coord_;
    int tax_;
    int distance_;
    //1 parent and N vassals -> N-ary tree
    std::vector<Town*> vassals_;
    Town* parent_;

    //hash unordered_set with town names
    struct town_name_hash{
        std::size_t operator()(const Town* t) const{
            return std::hash<std::string> ()(t->name_);
        }
    };

    // Roads / prg2
    std::unordered_set<Town*, town_name_hash> to_neighbours_;
    bool visited_;
    Colour colour;
    Town* prev;
    int range;
    int range_estimate;
};


//These were hash functions for hashing pairs in roads_(private)
/*
OK
struct pair_hash{
    inline std::size_t operator()(const std::pair<Town*, Town*> & a) const{
        return a.first->distance_*31+a.second->distance_;
    }
};

OK
struct pair_hash_2{
    inline std::size_t operator()(const std::pair<Town*, Town*> & a) const{
        return std::hash<std::string>()(a.first->name_)^std::hash<std::string>()(a.second->name_);
    }
};

OK
struct pair_hash_3{
    inline std::size_t operator()(const std::pair<Town*, Town*> & a) const{
        return std::hash<int>()(a.first->coord_.x)^std::hash<int>()(a.second->coord_.y);
    }
};

Not good one
struct pair_hash_4{
    inline std::size_t operator()(const std::pair<Town*, Town*> & a) const{
        return a.first->distance_^a.second->distance_;
    }
};*/

// BEST ONE
struct pair_hash_5{
    inline std::size_t operator()(const std::pair<Town*, Town*> & a) const{
        return ((a.first->distance_+a.second->distance_)*(a.first->distance_+a.second->distance_+1)/2)+a.second->distance_;
    }
};


class Datastructures
{
public:
    Datastructures();
    ~Datastructures();

    // Estimate of performance: θ(1)
    // Short rationale for estimate: Based on the cppreference, unordered_map.size is constant in complexity
    unsigned int town_count();

    // Estimate of performance: θ(n)
    // Short rationale for estimate:Clearing selected datastructures is linear
    void clear_all();

    // Estimate of performance:O(n)
    // Short rationale for estimate:id not found from main data structure
    bool add_town(TownID id, Name const& name, Coord coord, int tax);

    // Estimate of performance:O(n)/θ(1)
    // Short rationale for estimate:unordered_map.find constant on average
    Name get_town_name(TownID id);

    // Estimate of performance:O(n)/θ(1)
    // Short rationale for estimate:unordered_map.find constant on average
    Coord get_town_coordinates(TownID id);

    // Estimate of performance:O(n)/θ(1)
    // Short rationale for estimate:unordered_map.find constant on average
    int get_town_tax(TownID id);

    // Estimate of performance:θ(n)
    // Short rationale for estimate: For loop of all towns
    std::vector<TownID> all_towns();

    // Estimate of performance:θ(n)
    // Short rationale for estimate:For loop of all towns
    std::vector<TownID> find_towns(Name const& name);

    // Estimate of performance:O(n)/θ(1)
    // Short rationale for estimate:unordered_map.find constant on average
    bool change_town_name(TownID id, Name const& newname);

    // Estimate of performance:O(nlogn)
    // Short rationale for estimate:stl sort uses some sort of quicksort
    std::vector<TownID> towns_alphabetically();

    // Estimate of performance:θ(n)
    // Short rationale for estimate:For loop of all towns
    std::vector<TownID> towns_distance_increasing();

    // Estimate of performance:θ(1)
    // Short rationale for estimate:multimap.begin constant
    TownID min_distance();

    // Estimate of performance:θ(1)
    // Short rationale for estimate:multimap.rbegin constant
    TownID max_distance();

    // Estimate of performance:O(n)/θ(1)
    // Short rationale for estimate:unordered_map.find constant on average/vector.push pack constant
    bool add_vassalship(TownID vassalid, TownID masterid);

    // Estimate of performance:O(n)/θ(1)*k, where k is the number of direct vassals
    // Short rationale for estimate:Badest situation when id not found
    std::vector<TownID> get_town_vassals(TownID id);

    // Estimate of performance:O(n)/θ(1)*(s+1), where s is the number of parents
    // Short rationale for estimate:Badest situation when id not found
    std::vector<TownID> taxer_path(TownID id);

    // Non-compulsory phase 1 operations

    // Estimate of performance:O(n)
    // Short rationale for estimate:Badest situation when id not found
    bool remove_town(TownID id);

    // Estimate of performance:θ(nlogn)
    // Short rationale for estimate:logaritmic insert to map inside for loop
    std::vector<TownID> towns_nearest(Coord coord);

    // Estimate of performance:O(n*log(k)), k is the number of paths from id
    // Short rationale for estimate:logaritmic insert to map inside recursive function
    std::vector<TownID> longest_vassal_path(TownID id);

    // Estimate of performance:O(n)
    // Short rationale for estimate:Id not found/ root node to which everyone pays
    int total_net_tax(TownID id);


    // Phase 2 operations

    // V=vertices, E=edges
    // Estimate of performance:θ(V)
    // Short rationale for estimate:for looping all towns
    void clear_roads();

    // Estimate of performance:θ(E)
    // Short rationale for estimate: for looping all roads
    std::vector<std::pair<TownID, TownID>> all_roads();

    // Estimate of performance:O(V)
    // Short rationale for estimate:unordered_map/set operations linear at worst
    bool add_road(TownID town1, TownID town2);

    // Estimate of performance:θ(1)*k
    // Short rationale for estimate:Finding town is constant on average and k is the number of neighbortowns
    std::vector<TownID> get_roads_from(TownID id);

    // Estimate of performance:O(V+E)
    // Short rationale for estimate: DFS O(V+E) at worst
    std::vector<TownID> any_route(TownID fromid, TownID toid);

    // Non-compulsory phase 2 operations

    // Estimate of performance:O(log E)
    // Short rationale for estimate: Finding from set is linear at worst
    bool remove_road(TownID town1, TownID town2);

    // Estimate of performance:O(V+E)
    // Short rationale for estimate: BFS O(V+E) at worst
    std::vector<TownID> least_towns_route(TownID fromid, TownID toid);

    // Estimate of performance:O(V+E)
    // Short rationale for estimate:DFS O(V+E) at worst
    std::vector<TownID> road_cycle_route(TownID startid);

    // Estimate of performance:O((V+E)logV)
    // Short rationale for estimate:Dijkstra O((V+E)logV) at worst
    std::vector<TownID> shortest_route(TownID fromid, TownID toid);

    // Estimate of performance:O((V+E)logV)
    // Short rationale for estimate:Prim's algorithm O((V+E)logV) at worst
    Distance trim_road_network();

private:


    // Main data structure for towns
    std::unordered_map<TownID, Town> towns_;

    //Data structure for alphabetic order
    std::vector<Town*> alphaorder;
    std::vector<Town*>::iterator it;
    bool asorted_;
    static bool compareAlphabet(const Town* a, const Town* b);

    //Data structure for distance order
    std::multimap<int, Town*> distancemap;
    int count_distance(Coord first, Coord scnd);
    void all_roots(Town* town, std::vector<TownID> vec, std::multimap<int, std::vector<TownID>> &map);
    int all_taxes(Town* town);


    //------prg2-------
    //std::unordered_set<std::pair<Town*, Town*>, pair_hash_5> roads_;
    std::set<std::pair<Town*, Town*>> roads_;
    std::vector<Town*> DFS_some_route(Town* start, Town* end);
    std::vector<Town*> BFS(Town* start, Town* end);
    std::vector<Town*> DFS(Town* start);
    std::vector<Town*> A_star(Town* start, Town* end);
    bool A_star_Relax(Town* first, Town* scnd, Town* end);
    void initial_values();
    std::vector<std::pair<Town*,Town*>> Prims_Algorithm();

};

#endif // DATASTRUCTURES_HH
