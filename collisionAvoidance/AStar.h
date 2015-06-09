#ifndef ASTAR_H
#define ASTAR_H

#include <iostream>
#include <iomanip>
#include <queue>
#include <string>
#include <math.h>
#include <ctime>
#include <QTime>

#include "../debug_info.h"

#include "opencv2/core.hpp"

#define MAP_WIDTH 640
#define MAP_HEIGHT 320

// possible direction
const static int dir = 8;
static int dx[dir]={1, 1, 0, -1, -1, -1, 0, 1};
static int dy[dir]={0, 1, 1, 1, 0, -1, -1, -1};

class Node
{
public:
    // Current position
    int xPos, yPos;
    // total distance already travelled to reach the node
    int level;
    // priority = level + remaining distance estimate
    int priority;  // smaller: higher priority

    Node(int xpos, int ypos, int d, int p) {
        xPos = xpos;
        yPos = ypos;
        level = d;
        priority = p;
    }

    void Reset(int xpos, int ypos, int d, int p) {
        xPos = xpos;
        yPos = ypos;
        level = d;
        priority = p;
    }

    int getxPos() const { return xPos;}
    int getyPos() const { return yPos;}
    int getLevel() const {return level;}
    int getPriority() const {return priority;}

    // Estimation function for the remaining distance to the goal
    const int & estimate(const int & xDst, const int & yDst) {
        static int xd, yd, d;
        xd = xDst - xPos;
        yd = yDst - yPos;

        // Euclidian distance
        d = (int)(sqrt(pow(1.0*xd, 2) + pow(1.0*yd, 2)));

        return d;
    }

    void updatePriority(const int & xDst,  const int & yDst) {
        priority = level + estimate(xDst, yDst)*10;
    }

    // Give better priority to going straight instead of diagonally
    void nextLevel(const int & i) {
        level += (i%2 == 0 ? 10 : 14);  //when dir == 8
    }
};

class AStar
{
public:
    AStar();

    std::vector<std::pair<int, int> > PathFind2D(const int & xStart, const int & yStart, const int & xEnd, const int & yEnd);
//    std::string PathFind(const int & xStart, const int & yStart, const int & xEnd, const int & yEnd);
    void PathFind(const int & xStart, const int & yStart, const int & xEnd, const int & yEnd);

    void kernelDilation(cv::Point pt, cv::Size dilate_kernel);

    void resetMap();

    int map[MAP_WIDTH][MAP_HEIGHT];             // obstacle map     0: no obs, 1: obs

private:
    int open_nodes_map[MAP_WIDTH][MAP_HEIGHT];
    int closed_nodes_map[MAP_WIDTH][MAP_HEIGHT];
    int dir_map[MAP_WIDTH][MAP_HEIGHT];

    std::priority_queue<Node> pq[2];
    int pqi;
    Node* n0;
    Node* m0;
    int i, j, x, y, xdx, ydy;
    char c;
    std::string path;

    std::vector<std::pair<int, int> > path2D;

    QTime t_p;                          // process time of all exec.
};


#endif // ASTAR_H
