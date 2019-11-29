#include <ros/ros.h>
#include <queue>

#include <cost_map_ros/cost_map_ros.hpp>
#include <priority_point.h>
#include <global_planner.h>
#include <vector>
#include <math.h>

//////////////////////////////////////////////////////////////////////////////
//////////////////////////////// A* //////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
// double makePath_AStar(std::vector<cost_map::Index> &path, PriorityPoint start, PriorityPoint goal, std::string layer, cost_map::CostMap &cost_map, double grid_cost, bool only_cost)
// {
//   std::priority_queue<PriorityPoint, std::vector<PriorityPoint>, myCompare> openSet;
//   PriorityPoint neighbors[8];
//   PriorityPoint current;
//   size_t max_x = cost_map.getSize()[0];
//   size_t max_y = cost_map.getSize()[1];
//   double cost_so_far[max_x][max_y];
//   std::fill_n(*cost_so_far, max_x * max_y, std::numeric_limits<double>::max());
//   int come_from[max_x][max_y][2];

//   cost_so_far[start.x][start.y] = 0;
//   come_from[start.x][start.y][0] = start.x;
//   come_from[start.x][start.y][1] = start.y;
//   openSet.push(start);
//   while (!openSet.empty())
//   {
//     current = openSet.top();
//     openSet.pop();
//     if (current == goal)
//       break;

//     current.GetNeighbors(neighbors);
//     for (int i = 0; i < 8; i++)
//     {
//       if (!neighbors[i].OnMap(max_x, max_y))
//       {
//         continue;
//       }
//       double neighbor_price = cost_map.at(layer, cost_map::Index({neighbors[i].x, neighbors[i].y})) + neighbors[i].priority;
//       double current_cost = cost_so_far[current.x][current.y];
//       double new_cost = current_cost + neighbor_price;
//       if (new_cost < cost_so_far[neighbors[i].x][neighbors[i].y])
//       {
//         cost_so_far[neighbors[i].x][neighbors[i].y] = new_cost;
//         neighbors[i].priority = HeuristicEvclid(neighbors[i], goal, grid_cost) + new_cost;
//         openSet.push(neighbors[i]);
//         come_from[neighbors[i].x][neighbors[i].y][0] = current.x;
//         come_from[neighbors[i].x][neighbors[i].y][1] = current.y;
//       }
//     }
//   }
//   if (only_cost)
//   {
//     return cost_so_far[current.x][current.y];
//   }
//   path.clear();
//   PriorityPoint temp_point;
//   while (current != start)
//   {
//     path.push_back({current.x, current.y});
//     temp_point.x = come_from[current.x][current.y][0];
//     temp_point.y = come_from[current.x][current.y][1];
//     current = temp_point;
//   }
//   path.push_back({current.x, current.y});
//   return 0;
// }

//////////////////////////////////////////////////////////////////////////////
//////////////////////////// Theta* //////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////

float cost_so_far[300][200];
PriorityPoint neighbors[8];
PriorityPoint current;
int come_from[300][200][2];

double makePath_ThetaStar(std::vector<cost_map::Index> &path, PriorityPoint start, PriorityPoint goal, std::string layer, cost_map::CostMap &cost_map, double grid_cost, bool only_cost)
{
  std::priority_queue<PriorityPoint, std::vector<PriorityPoint>, myCompare> openSet;
  size_t max_x = cost_map.getSize()[0];
  size_t max_y = cost_map.getSize()[1];
  std::fill_n(*cost_so_far, max_x * max_y, std::numeric_limits<float>::max());
  cost_so_far[start.x][start.y] = 0;
  come_from[start.x][start.y][0] = start.x;
  come_from[start.x][start.y][1] = start.y;
  openSet.push(start);
  grid_cost=5;
  while (!openSet.empty())
  {
    current = openSet.top();
    openSet.pop();
    if (current == goal)
    {
      break;
    }
    current.GetNeighbors(neighbors);
    float current_cost = cost_so_far[current.x][current.y];
    int parent_carent[2] ={come_from[current.x][current.y][0], come_from[current.x][current.y][1]};
    for (int i = 0; i < 8; i++)
    {
      if (!neighbors[i].OnMap(max_x, max_y))
      {
        continue;
      }
      bool onLine = lineOfSight(parent_carent[0], parent_carent[1], neighbors[i].x, neighbors[i].y, cost_map, layer, grid_cost + 1);
      if (onLine)
      {
        float new_cost = cost_so_far[parent_carent[0]][parent_carent[1]] + HeuristicEvclid(parent_carent, neighbors[i], grid_cost*10);
        if (new_cost < cost_so_far[neighbors[i].x][neighbors[i].y])
        {
          cost_so_far[neighbors[i].x][neighbors[i].y] = new_cost;
	        neighbors[i].priority = HeuristicEvclid(neighbors[i], goal, grid_cost*10) + new_cost;
          openSet.push(neighbors[i]);
          come_from[neighbors[i].x][neighbors[i].y][0] = parent_carent[0];
          come_from[neighbors[i].x][neighbors[i].y][1] = parent_carent[1];
        }
      }
      else
      {
        float neighbor_price = cost_map.at(layer, cost_map::Index({neighbors[i].x, neighbors[i].y})) + neighbors[i].priority;
        float new_cost = current_cost + neighbor_price;
        if (new_cost < cost_so_far[neighbors[i].x][neighbors[i].y])
        {
          cost_so_far[neighbors[i].x][neighbors[i].y] = new_cost;
          neighbors[i].priority =HeuristicEvclid(neighbors[i], goal, grid_cost*10) + new_cost;
          openSet.push(neighbors[i]);
          come_from[neighbors[i].x][neighbors[i].y][0] = current.x;
          come_from[neighbors[i].x][neighbors[i].y][1] = current.y;
        }
      }
    }
  }

  if (only_cost)
  {
    return cost_so_far[current.x][current.y];
  }
  path.clear();
  PriorityPoint temp_point;
  
  while (current != start)
  {
    path.push_back({current.x, current.y});
    temp_point.x = come_from[current.x][current.y][0];
    temp_point.y = come_from[current.x][current.y][1];
    current = temp_point;
  }
  path.push_back({current.x, current.y});
  return 0;
}

////////////////////////////// boost //////////////////////////////////////////
double makePath_BoostThetaStar(std::vector<cost_map::Index> &path, PriorityPoint start, PriorityPoint goal, std::string layer, cost_map::CostMap &cost_map, double grid_cost, bool only_cost)
{
  std::priority_queue<PriorityPoint, std::vector<PriorityPoint>, myCompare> openSet;
  size_t max_x = cost_map.getSize()[0];
  size_t max_y = cost_map.getSize()[1];
  std::fill_n(*cost_so_far, max_x * max_y, std::numeric_limits<float>::max());
  cost_so_far[start.x][start.y] = 0;
  come_from[start.x][start.y][0] = start.x;
  come_from[start.x][start.y][1] = start.y;
  openSet.push(start);
  grid_cost=5;
  unsigned int dist = 2;
  while (!openSet.empty())
  {
    current = openSet.top();
    openSet.pop();
    if (isNear(current,goal,dist))
    {
      break;
    }
    current.GetNeighbors(neighbors,dist);
    float current_cost = cost_so_far[current.x][current.y];
    int parent_carent[2] ={come_from[current.x][current.y][0], come_from[current.x][current.y][1]};
    for (int i = 0; i < 8; i++)
    {
      if (!neighbors[i].OnMap(max_x, max_y))
      {
        continue;
      }
      // this is first candidate for optimisation
      bool onLine = lineOfSight(parent_carent[0], parent_carent[1], neighbors[i].x, neighbors[i].y, cost_map, layer, grid_cost + 1);
      // this is first candidate for optimisation
      if (onLine)
      {
        float new_cost = cost_so_far[parent_carent[0]][parent_carent[1]] + HeuristicEvclid(parent_carent, neighbors[i], grid_cost*2);
        if (new_cost < cost_so_far[neighbors[i].x][neighbors[i].y])
        {
          cost_so_far[neighbors[i].x][neighbors[i].y] = new_cost;
	        neighbors[i].priority = HeuristicEvclid(neighbors[i], goal, grid_cost*2) + new_cost;
          openSet.push(neighbors[i]);
          come_from[neighbors[i].x][neighbors[i].y][0] = parent_carent[0];
          come_from[neighbors[i].x][neighbors[i].y][1] = parent_carent[1];
        }
      }
      else
      {
        float neighbor_price = cost_map.at(layer, cost_map::Index({neighbors[i].x, neighbors[i].y})) + neighbors[i].priority;
        float new_cost = current_cost + neighbor_price;
        if (new_cost < cost_so_far[neighbors[i].x][neighbors[i].y])
        {
          cost_so_far[neighbors[i].x][neighbors[i].y] = new_cost;
          neighbors[i].priority =HeuristicEvclid(neighbors[i], goal, grid_cost*2) + new_cost;
          openSet.push(neighbors[i]);
          come_from[neighbors[i].x][neighbors[i].y][0] = current.x;
          come_from[neighbors[i].x][neighbors[i].y][1] = current.y;
        }
      }
    }
  }

  if (only_cost)
  {
    return cost_so_far[current.x][current.y];
  }
  path.clear();
  PriorityPoint temp_point;
  
  while (current != start)
  {
    path.push_back({current.x, current.y});
    temp_point.x = come_from[current.x][current.y][0];
    temp_point.y = come_from[current.x][current.y][1];
    current = temp_point;
  }
  path.push_back({current.x, current.y});
  return 0;
}


bool lineOfSight(int i1, int j1, int i2, int j2, cost_map::CostMap &cost_map, std::string layer, int minimal_cost)
{
  cost_map::Index p1 = {i1, j1};
  cost_map::Index p2 = {i2, j2};
  for (cost_map::LineIterator iterator(cost_map, p1, p2); !iterator.isPastEnd(); ++iterator)
  {
    if (cost_map.at(layer, *iterator) >= minimal_cost)
    {
      return false;
    }
  }
  return true;
}


//////////////////////////////////////////////////////////////////////////////
////////////////////////////// support ///////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
bool isNear(PriorityPoint &p1,PriorityPoint &p2,unsigned int dist)
{
  unsigned int a = abs(p1.x - p2.x);
  unsigned int b = abs(p1.y - p2.y);
  return (a + b)<dist;

}


//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////
//////////////////////////////// Heuristics //////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
float HeuristicEvclid(const PriorityPoint &source, const PriorityPoint &target, const double &unit_cost)
{
  return sqrt(pow(source.x - target.x, 2) + pow(source.y - target.y, 2)) * unit_cost;
}

float HeuristicEvclid(int* source, const PriorityPoint &target, const double &unit_cost)
{
  return sqrt(pow(source[0] - target.x, 2) + pow(source[1] - target.y, 2)) * unit_cost;
}


unsigned int HeuristicChebishev(const PriorityPoint &source, const PriorityPoint &target, const double &unit_cost)
{
  unsigned int a = abs(source.x - target.x);
  unsigned int b = abs(source.y - target.y);
  return a > b ? a * unit_cost : b * unit_cost;
}
unsigned int HeuristicManhetten(const PriorityPoint &source, const PriorityPoint &target, const double &unit_cost)
{
  return (abs(source.x - target.x) + abs(source.y - target.y)) * unit_cost;
}
unsigned int HeuristicManhetten(int* source, const PriorityPoint &target, const double &unit_cost)
{
  return (abs(source[0] - target.x) + abs(source[1] - target.y)) * unit_cost;
}
//////////////////////////////////////////////////////////////////////////////
/////////////////////////////////// Path /////////////////////////////////////
//////////////////////////////////////////////////////////////////////////////
