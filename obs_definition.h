#include <vector> 

 // Main structures for obstacles using vertex = std::pair<float, float>;
using vertex = std::pair<double, double>;
using polygon = std::vector<vertex>;
using obstacles = std::vector<polygon>;

polygon createPol();
obstacles createObs(); 
polygon centroid_calculation(obstacles &); 
obstacles island_definition(vertex, obstacles &, polygon &, double); 
bool SAT_collision(polygon &, obstacles &);
bool SAT_collision_circle(vertex, double, obstacles &);
