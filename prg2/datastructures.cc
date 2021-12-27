// Datastructures.cc

#include "datastructures.hh"

#include <random>

#include <cmath>

std::minstd_rand rand_engine; // Reasonably quick pseudo-random generator

template <typename Type>
Type random_in_range(Type start, Type end)
{
    auto range = end-start;
    ++range;

    auto num = std::uniform_int_distribution<unsigned long int>(0, range-1)(rand_engine);

    return static_cast<Type>(start+num);
}

// Modify the code below to implement the functionality of the class.
// Also remove comments from the parameter names when you implement
// an operation (Commenting out parameter name prevents compiler from
// warning about unused parameters on operations you haven't yet implemented.)

Datastructures::Datastructures()
{
    asorted_ = false;
}

Datastructures::~Datastructures()
{
    // Write any cleanup you need here
}

unsigned int Datastructures::town_count()
{
    return towns_.size();
}

void Datastructures::clear_all()
{
    towns_.clear();
    alphaorder.clear();
    distancemap.clear();
    roads_.clear();
}

bool Datastructures::add_town(TownID id, const Name &name, Coord coord, int tax)
{
    if(towns_.find(id) != towns_.end()){
        return false;
    }
    else{
        Town t;

        //pr1
        t.id_ = id;
        t.name_ = name;
        t.coord_ = coord;
        t.tax_ = tax;
        t.vassals_ ={};
        t.parent_ = nullptr;
        t.distance_ = count_distance(ORIGO,t.coord_);

        //prg2
        t.to_neighbours_ = {};
        t.visited_ = false;
        t.prev = nullptr;
        t.colour = WHITE;
        t.range = std::numeric_limits<int>::max();
        t.range_estimate = std::numeric_limits<int>::max();

        // main datastructure
        towns_.insert({id,t});

        // structure for alphabetic order
        alphaorder.push_back(&towns_[id]);
        asorted_ = false;

        // structure for distance order
        distancemap.insert({t.distance_, &towns_[id]});
        return true;
    }
}

Name Datastructures::get_town_name(TownID id)
{
    auto find = towns_.find(id);
    if(find != towns_.end())
    {
        return find->second.name_;
    }
    else
    {
        return NO_NAME;
    }
}

Coord Datastructures::get_town_coordinates(TownID id)
{
    auto find = towns_.find(id);
    if(find != towns_.end())
    {
        return find->second.coord_;
    }
    else
    {
        return NO_COORD;
    }
}

int Datastructures::get_town_tax(TownID id)
{
    auto find = towns_.find(id);
    if(find != towns_.end())
    {
        return find->second.tax_;
    }
    else
    {
        return NO_VALUE;
    }
}
std::vector<TownID> Datastructures::all_towns()
{
    std::vector<TownID> v;
    for(auto& city: towns_){
        v.push_back(city.second.id_);
    }
    return v;
}

std::vector<TownID> Datastructures::find_towns(const Name &name)
{
    std::vector<TownID> v;
    for(auto& city : towns_){
        if (city.second.name_ == name){
            v.push_back(city.second.id_);
        }
    }
    return v;
}

bool Datastructures::change_town_name(TownID id, const Name &newname)
{
    auto find = towns_.find(id);
    if(find != towns_.end())
    {
        find->second.name_ = newname;
        asorted_ = false;
        return true;
    }
    else
    {
        return false;
    }
}

std::vector<TownID> Datastructures::towns_alphabetically()
{
    std::vector<TownID> v;
    if(!asorted_)
        {
            std::sort(alphaorder.begin(), alphaorder.end(), compareAlphabet);

            asorted_ = true;
        }
    for(auto city:alphaorder)
    {
        v.push_back(city->id_);
    }
   return v;
}

std::vector<TownID> Datastructures::towns_distance_increasing()
{
    std::vector<TownID> v;
    for(auto city:distancemap)
    {
        v.push_back(city.second->id_);
    }
    return v;
}

TownID Datastructures::min_distance()
{
    if(towns_.size() ==0){return NO_TOWNID;}
    return distancemap.begin()->second->id_;
}

TownID Datastructures::max_distance()
{
    if(towns_.size() ==0){return NO_TOWNID;}
    return distancemap.rbegin()->second->id_;
}

bool Datastructures::add_vassalship(TownID vassalid, TownID masterid)
{
     auto vassal_find = towns_.find(vassalid);
     auto master_find = towns_.find(masterid);

     // both ids must found
     if(vassal_find != towns_.end() && master_find != towns_.end()){
         // vassal cant have parent already
         if(vassal_find->second.parent_ != nullptr){
             return false;
         }else{
             master_find->second.vassals_.push_back(&vassal_find->second);
             vassal_find->second.parent_ = &master_find->second;
             return true;
         }
     }else{
         return false;
     }
}

std::vector<TownID> Datastructures::get_town_vassals(TownID id)
{
    std::vector<TownID> v;
    auto find = towns_.find(id);
    if(find != towns_.end()){
        for(auto& vassal: find->second.vassals_){
            v.push_back(vassal->id_);
        }
    }else{
        v.push_back(NO_TOWNID);
    }
    return v;
}

std::vector<TownID> Datastructures::taxer_path(TownID id)
{
    auto find = towns_.find(id);
    if(find != towns_.end())
    {
        if(find->second.parent_ !=nullptr){
            //recursion for parent
            std::vector<TownID> a = taxer_path(find->second.parent_->id_);
            a.insert(a.begin(),id);
            return a;
        }else{
            std::vector<TownID> v;
            v.push_back(id);
            return v;
        }
    }else
    {
        std::vector<TownID> v;
        v.push_back(NO_TOWNID);
        return v;
    }
}

bool Datastructures::remove_town(TownID id)
{
    auto find = towns_.find(id);
    if(find != towns_.end())
    {
        //prg1
        // parent and vassals
        if(find->second.parent_ !=nullptr && !find->second.vassals_.empty()){

            for(auto& vassal:find->second.vassals_){
                find->second.parent_->vassals_.push_back(vassal);
                vassal->parent_ = find->second.parent_;
            }
            auto it_ = std::find(find->second.parent_->vassals_.begin(), find->second.parent_->vassals_.end(), &find->second);
            find->second.parent_->vassals_.erase(it_);

        // no parent and vassals
        }else if (find->second.parent_ ==nullptr && !find->second.vassals_.empty()) {
            for(auto& vassal:find->second.vassals_){
                vassal->parent_ = nullptr;
            }
            find->second.vassals_.clear();

        // parent and no vassals
        }else if(find->second.parent_ !=nullptr && find->second.vassals_.empty()){
            auto it_ = std::find(find->second.parent_->vassals_.begin(), find->second.parent_->vassals_.end(), &find->second);
            find->second.parent_->vassals_.erase(it_);
        }

        //prg2
        if(!find->second.to_neighbours_.empty()){

            for(auto t: find->second.to_neighbours_ ){
                remove_road(id, t->id_);
            }
        }

        //removing town from main datastructures
        it = std::find(alphaorder.begin(), alphaorder.end(), &find->second);
        alphaorder.erase(it);
        distancemap.erase(find->second.distance_);
        towns_.erase(id);
        return true;

    }else
    {
        return false;
    }
}

std::vector<TownID> Datastructures::towns_nearest(Coord coord)
{
    std::multimap<int,std::string>m;
    //counts distance for each town to coord and add to multimap{distance,id}
    for(auto& town:towns_){
        int distance = count_distance(town.second.coord_,coord);
        m.insert({distance, town.second.id_});
    }
    std::vector<TownID> v;
    for(auto& i:m){
        v.push_back(i.second);
    }
    return v;
}

std::vector<TownID> Datastructures::longest_vassal_path(TownID id)
{
    if(towns_.find(id) == towns_.end()){std::vector<TownID> v ={NO_TOWNID}; return v;}
    std::vector<TownID> a;
    std::multimap<int, std::vector<TownID>> map;

    //makes all paths for town into map like {path_size, path}
    all_roots(&towns_[id], a, map);

    //Map in asc order so lets take the last key value
    return map.rbegin()->second;

}

int Datastructures::total_net_tax(TownID id)
{
    if(towns_.find(id) == towns_.end()){return NO_VALUE;}

    Town town = towns_[id];
    //if else for four different parent/vassal cases
    if(town.parent_==nullptr){
        if(town.vassals_.empty()){
            return town.tax_;
        }else{
            return all_taxes(&town);
        }
    }else{
        if(town.vassals_.empty()){
            return town.tax_-floor(town.tax_/10);
        }else{
            int a =all_taxes(&town);
            return a-floor(a/10);
        }
    }
}

int Datastructures::all_taxes(Town *town){

    int sum = town->tax_;
    for(unsigned int i=0; i<town->vassals_.size(); ++i){
        sum += floor(all_taxes(town->vassals_[i])/10);
    }
    return sum;
}

//counts all different paths for *town and insert them to &map
void Datastructures::all_roots(Town *town, std::vector<TownID> vec, std::multimap<int, std::vector<TownID>> &map)
{
    if(!town){return;}
    vec.push_back(town->id_);

    if(town->vassals_.empty()){
        map.insert({vec.size(), vec});
        vec.pop_back();
        return;
    }
    for(unsigned int i=0;i<town->vassals_.size();i++){
        all_roots(town->vassals_[i], vec, map);
    }
}



int Datastructures::count_distance(Coord first, Coord scnd)
{
    int distance = floor(sqrt((first.x-scnd.x)*(first.x-scnd.x)+(first.y-scnd.y)*(first.y-scnd.y)));
    return distance;
}

bool Datastructures::compareAlphabet(const Town* a, const Town* b){
    return a->name_<b->name_;
}


//
// Phase 2 operations
//

void Datastructures::clear_roads()
{
    for(auto& town: towns_){
        town.second.to_neighbours_.clear();
    }
    roads_.clear();
}

std::vector<std::pair<TownID, TownID>> Datastructures::all_roads()
{
    std::vector<std::pair<TownID, TownID>> v ={};
    for(auto& pair: roads_){
        v.push_back(std::pair(pair.first->id_, pair.second->id_));
    }
    return v;
}

//adds road if both ids found and there is not already road
bool Datastructures::add_road(TownID town1, TownID town2)
{
    // not adding road to itself
    if(town1 == town2){return false;}

    auto first_find = towns_.find(town1);
    auto scnd_find = towns_.find(town2);

    if(first_find != towns_.end() && scnd_find != towns_.end()){
        auto a = first_find->second.to_neighbours_.find(&towns_[town2]);
        //there is already a road
        if(a != first_find->second.to_neighbours_.end()){
            return false;
        }
        // this order check is for all_roads. Inserting to roads_ in alphabetic order
        if(town1<town2){
            roads_.insert(std::pair(&first_find->second,&scnd_find->second));
        }else{
            roads_.insert(std::pair(&scnd_find->second,&first_find->second));
        }
        //inserting to_neighbours of both towns
        first_find->second.to_neighbours_.insert(&scnd_find->second);
        scnd_find->second.to_neighbours_.insert(&first_find->second);

        return true;
    }else{
        return false;
    }
}

//returns all immediate ids/towns where u can get from town
std::vector<TownID> Datastructures::get_roads_from(TownID id)
{
    std::vector<TownID> v;
    auto find = towns_.find(id);
    if(find != towns_.end()){

        for (auto& town: find->second.to_neighbours_){
            v.push_back(town->id_);
        }
        return v;

    }else{
        v.push_back(NO_TOWNID);
        return v;
    }
}

//returns any route between two towns. Search implemented with DFS
std::vector<TownID> Datastructures::any_route(TownID fromid, TownID toid)
{
    std::vector<TownID> v = {};
    //not starting the search if ids are same
    if(fromid == toid){return v;}
    //initial town data for DFS
    initial_values();

    auto from_find = towns_.find(fromid);
    auto to_find = towns_.find(toid);
    //both ids must found
    if(from_find != towns_.end() && to_find != towns_.end()){

        std::vector<Town*> a;
        a = DFS_some_route(&towns_[fromid], &towns_[toid]);
        //turning vector in reverse order
        for(unsigned int i = a.size()-1;a.size()>i;--i){
            v.push_back(a[i]->id_);
        }
    }else{
        v.push_back(NO_TOWNID);
        return v;
    }
    return v;
}

bool Datastructures::remove_road(TownID town1, TownID town2)
{
    //Finding both ids
    auto first_find = towns_.find(town1);
    auto scnd_find = towns_.find(town2);

    if(first_find != towns_.end() && scnd_find != towns_.end()){
        //If there is road, town2 must be in to_neighbours of town1.(and vise verca)
        auto neighbour_find = first_find->second.to_neighbours_.find(&towns_[town2]);
        if (neighbour_find ==first_find->second.to_neighbours_.end()){
            return false;
        }else{
            //erasing from to_neighbours of both towns
            first_find->second.to_neighbours_.erase(&towns_[town2]);
            scnd_find->second.to_neighbours_.erase(&towns_[town1]);
            //erasing from roads_
            if(town1<town2){
                auto find = roads_.find(std::pair(&towns_[town1],&towns_[town2]));
                roads_.erase(find);
            }else{
                auto find = roads_.find(std::pair(&towns_[town2],&towns_[town1]));
                roads_.erase(find);
            }
            return true;
        }
    }else{
        return false;
    }
}

//returns route (with least towns) between two towns. Search implemented with BFS
std::vector<TownID> Datastructures::least_towns_route(TownID fromid, TownID toid)
{   
    std::vector<TownID> v = {};
    //not starting the search if ids are same
    if(fromid == toid){return v;}
    //initial town data for BFS
    initial_values();

    auto from_find = towns_.find(fromid);
    auto to_find = towns_.find(toid);  

    if(from_find != towns_.end() && to_find != towns_.end()){

        std::vector<Town*> a;
        a = BFS(&towns_[fromid], &towns_[toid]);
        //turning vector in reverse order
        for(unsigned int i = a.size()-1;a.size()>i;--i){
            v.push_back(a[i]->id_);
        }

    }else{
        v.push_back(NO_TOWNID);
        return v;
    }
    return v;
}

//Finds cycles in graph.Search implemented with DFS
std::vector<TownID> Datastructures::road_cycle_route(TownID startid)
{
    //initial town data for DFS
    initial_values();

    auto find = towns_.find(startid);
    std::vector<TownID> v;

    if(find != towns_.end()){

        std::vector<Town*> a;
        a = DFS(&towns_[startid]);
        //turning vector in reverse order
        for(unsigned int i = a.size()-1;a.size()>i;--i){
            v.push_back(a[i]->id_);
        }
    }else{
        v.push_back(NO_TOWNID);
        return v;
    }
    return v;
}

// Finds shortest route in weighted graph. Search implemented with A*(Dijkstra)
std::vector<TownID> Datastructures::shortest_route(TownID fromid, TownID toid)
{
    std::vector<TownID> v = {};
    //not starting the search if ids are same
    if(fromid == toid){return v;}
    //initial town data for A*
    initial_values();

    auto from_find = towns_.find(fromid);
    auto to_find = towns_.find(toid);

    if(from_find != towns_.end() && to_find != towns_.end()){
        std::vector<Town*> a;
        a = A_star(&towns_[fromid],&towns_[toid]);
        for(unsigned int i = a.size()-1;a.size()>i;--i){
            v.push_back(a[i]->id_);
        }
    }else{
        v.push_back(NO_TOWNID);
        return v;
    }
    return v;

}

//Returns minium spanning tree. Implemented with Prim's algorithm
Distance Datastructures::trim_road_network()
{
    //initial town data for Prim's
    initial_values();

    //Vector v includes minium spanning tree
    std::vector<std::pair<Town*,Town*>> v;
    v = Prims_Algorithm();
    //Clearing all roads
    clear_roads();

    Distance minium_cost =0;
    //Adding roads with for loop and count the minium distance with it
    for(auto & pair: v){
        minium_cost += count_distance(pair.first->coord_, pair.second->coord_);
        add_road(pair.first->id_, pair.second->id_);
    }
    return minium_cost;
}

//BFS with bool implementation
std::vector<Town*> Datastructures::BFS(Town *start, Town *end){
    std::queue<Town*> Queue;
    bool reached_end = false;
    start->visited_ = true;
    Queue.push(start);
    while(!Queue.empty() &&  !reached_end){
        Town* current_town =Queue.front();
        Queue.pop();
        for (auto node: current_town->to_neighbours_){
            if(!node->visited_){
                node->visited_ = true;
                Queue.push(node);
                node->prev = current_town;

                if(node == end){
                    reached_end = true;
                    break;
                }
            }
        }
    }
    if (reached_end == false){
        return {};
    }
    std::vector<Town*> v;
    Town* t = end;
    while(t != nullptr){
        v.push_back(t);
        t = t->prev;
    }
    return v;
}

//DFS with colour implementation
std::vector<Town*> Datastructures::DFS(Town *start){
    std::stack<Town*> Stack;
    Stack.push(start);
    while(!Stack.empty()){

        Town* current = Stack.top();
        Stack.pop();
        if(current->colour == WHITE){
            current->colour = GRAY;
            Stack.push(current);
            for(auto& node : current->to_neighbours_){
                if(node->colour == WHITE){
                    Stack.push(node);
                    node->prev = current;
                }else if(node->colour==GRAY && current->prev != node){

                    std::vector<Town*> v;
                    v.push_back(node);
                    Town* t = current;
                    while(t != nullptr){
                        v.push_back(t);
                        t = t->prev;
                    }
                    return v;
                }
            }
        }else{
            current->colour = BLACK;
        }
    }
    return {};
}

//DFS algorithm for looking some route, not maybe route with least towns
std::vector<Town*> Datastructures::DFS_some_route(Town *start, Town *end)
{
    std::stack<Town*> Stack;
    bool reached_end = false;
    start->visited_ = true;
    Stack.push(start);
    while(!Stack.empty() &&  !reached_end){
        Town* current_town =Stack.top();
        Stack.pop();
        for (auto node: current_town->to_neighbours_){
            if(!node->visited_){
                node->visited_ = true;
                Stack.push(node);
                node->prev = current_town;

                if(node == end){
                    reached_end = true;
                    break;
                }
            }
        }
    }
    if (reached_end == false){
        return {};
    }
    std::vector<Town*> v;
    Town* t = end;
    while(t != nullptr){
        v.push_back(t);
        t = t->prev;
    }
    return v;
}

// Dijkstra for finding the shortest route in weighted graph. Implemented with colours
std::vector<Town*> Datastructures::A_star(Town *start, Town *end){

    std::priority_queue<std::pair<int, Town*>> PrioQ;

    start->colour = GRAY;
    start->range = 0;
    PrioQ.push(std::pair(start->range, start));

    while(!PrioQ.empty()){
        Town* current = PrioQ.top().second;
        PrioQ.pop();

        if(current==end){
            std::vector<Town*> v;
            Town* t = current;
            while(t != nullptr){
                v.push_back(t);
                t = t->prev;
            }
            return v;
        }

        if(current->colour == BLACK){
            continue;
        }

        for(auto& neighbour: current->to_neighbours_){
            bool a = A_star_Relax(current, neighbour, end);

            if(neighbour->colour == WHITE){
                neighbour->colour = GRAY;
                PrioQ.push(std::pair(-1*(neighbour->range_estimate), neighbour));

            }else{
                if(a){
                    PrioQ.push(std::pair(-1*(neighbour->range), neighbour));
                }
            }
        }
        current->colour = BLACK;
    }
    return {};
}

//Check and update towns in PrioQ if needed
bool Datastructures::A_star_Relax(Town *first, Town *scnd, Town *end){

    if(scnd->range > first->range + count_distance(first->coord_,scnd->coord_)){
        scnd->range=first->range + count_distance(first->coord_,scnd->coord_);
        scnd->range_estimate = scnd->range + count_distance(scnd->coord_, end->coord_);
        scnd->prev = first;
        return true;
    }
    return false;
}

// Initiales all data needed for searching algorithms.
void Datastructures::initial_values(){
    for(auto& it: towns_){
        it.second.visited_ = false;
        it.second.prev = nullptr;
        it.second.colour = WHITE;
        it.second.range = std::numeric_limits<int>::max();
        it.second.range_estimate = std::numeric_limits<int>::max();
    }
}

//Finds the minium weight spanning tree
std::vector<std::pair<Town*,Town*>> Datastructures::Prims_Algorithm(){

    std::vector<std::pair<Town*,Town*>> MST;
    std::priority_queue<std::pair<int, Town*>> PrioQ;

    Town* t = &towns_.begin()->second;
    t->range =0;
    PrioQ.push(std::make_pair(t->range,t));

    while(!PrioQ.empty()){
        Town* current = PrioQ.top().second;
        PrioQ.pop();

        if(current->prev != nullptr && current->visited_ == false){
            MST.push_back(std::make_pair(current, current->prev));
        }
        current->visited_ = true;
        for(auto& node : current->to_neighbours_){
            if(node->visited_ == false && node->range > count_distance(current->coord_, node->coord_)){
                node->prev = current;
                node->range = count_distance(current->coord_, node->coord_);
                PrioQ.push(std::pair(-1*(node->range), node));
            }
        }
    }

    return MST;
}

