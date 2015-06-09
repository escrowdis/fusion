#include "AStar.h"

using namespace std;

// Determine priority (in the priority queue)
bool operator < (const Node & a, const Node & b) {
  return a.getPriority() > b.getPriority();
}

AStar::AStar()
{
    m0 = new Node(0, 0, 0, 0);
    n0 = new Node(0, 0, 0, 0);
}

std::vector<std::pair<int, int> > AStar::PathFind2D(const int & xStart, const int & yStart, const int & xEnd, const int & yEnd)
{
#ifdef debug_info_ca_astar
    t_p.restart();
#endif

    PathFind(xStart, yStart, xEnd, yEnd);

#ifdef debug_info_ca_astar
    std::cout<<"A* proc. time = "<<t_p.elapsed()<<std::endl;
#endif
    return path2D;
}

//std::string AStar::PathFind(const int & xStart, const int & yStart, const int & xEnd, const int & yEnd)
void AStar::PathFind(const int & xStart, const int & yStart, const int & xEnd, const int & yEnd)
{
    pqi = 0;
    // reset the node maps
    for(y=0;y<MAP_HEIGHT;y++)
    {
        for(x=0;x<MAP_WIDTH;x++)
        {
            closed_nodes_map[x][y]=0;
            open_nodes_map[x][y]=0;
        }
    }

    // create the start node and push into list of open nodes
    n0->Reset(xStart, yStart, 0, 0);
    n0->updatePriority(xEnd, yEnd);
    pq[pqi].push(*n0);
    open_nodes_map[x][y]=n0->getPriority(); // mark it on the open nodes map

    // A* search
    path = "";
    path2D.clear();
    while(!pq[pqi].empty())
    {
        // get the current node w/ the highest priority
        // from the list of open nodes
        //        n0=new Node( pq[pqi].top().getxPos(), pq[pqi].top().getyPos(),
        //                     pq[pqi].top().getLevel(), pq[pqi].top().getPriority());
        n0->Reset( pq[pqi].top().getxPos(), pq[pqi].top().getyPos(),
                     pq[pqi].top().getLevel(), pq[pqi].top().getPriority());

        x = n0->getxPos();
        y = n0->getyPos();

        pq[pqi].pop(); // remove the node from the open list
        open_nodes_map[x][y]=0;
        // mark it on the closed nodes map
        closed_nodes_map[x][y]=1;

        // quit searching when the goal state is reached
        //if((*n0).estimate(xEnd, yEnd) == 0)
        if(x==xEnd && y==yEnd)
        {
            // generate the path from finish to start
            // by following the directions
            while(!(x==xStart && y==yStart))
            {
                j=dir_map[x][y];
                c='0'+(j+dir/2)%dir;
                x+=dx[j];
                y+=dy[j];
                path2D.push_back(std::pair<int, int>(x, y));
            }

            // garbage collection
//            delete n0;
            // empty the leftover nodes
            while(!pq[pqi].empty()) pq[pqi].pop();
//            return path;
        }
        // generate moves (child nodes) in all possible directions
        for(i=0;i<dir;i++)
        {
            xdx=x+dx[i]; ydy=y+dy[i];

            if(!(xdx<0 || xdx>MAP_WIDTH-1 || ydy<0 || ydy>MAP_HEIGHT-1 || map[xdx][ydy]==1
                 || closed_nodes_map[xdx][ydy]==1))
            {
                // generate a child node
//                m0=new Node( xdx, ydy, n0->getLevel(),
//                             n0->getPriority());
                m0->Reset( xdx, ydy, n0->getLevel(),
                             n0->getPriority());
                m0->nextLevel(i);
                m0->updatePriority(xEnd, yEnd);

                // if it is not in the open list then add into that
                if(open_nodes_map[xdx][ydy]==0)
                {
                    open_nodes_map[xdx][ydy]=m0->getPriority();
                    pq[pqi].push(*m0);
                    // mark its parent node direction
                    dir_map[xdx][ydy]=(i+dir/2)%dir;
                }
                else if(open_nodes_map[xdx][ydy]>m0->getPriority())
                {
                    // update the priority info
                    open_nodes_map[xdx][ydy]=m0->getPriority();
                    // update the parent direction info
                    dir_map[xdx][ydy]=(i+dir/2)%dir;

                    // replace the node
                    // by emptying one pq to the other one
                    // except the node to be replaced will be ignored
                    // and the new node will be pushed in instead
                    while(!(pq[pqi].top().getxPos()==xdx &&
                            pq[pqi].top().getyPos()==ydy))
                    {
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();
                    }
                    pq[pqi].pop(); // remove the wanted node

                    // empty the larger size pq to the smaller one
                    if(pq[pqi].size()>pq[1-pqi].size()) pqi=1-pqi;
                    while(!pq[pqi].empty())
                    {
                        pq[1-pqi].push(pq[pqi].top());
                        pq[pqi].pop();
                    }
                    pqi=1-pqi;
                    pq[pqi].push(*m0); // add the better node instead
                }
//                else delete m0; // garbage collection
            }
        }
//        delete n0; // garbage collection
    }
//    delete n0, m0;
//    return ""; // no route found
}

void AStar::resetMap()
{
    for (int r = 0 ; r < MAP_HEIGHT; r++) {
        for (int c = 0; c < MAP_WIDTH; c++) {
            map[c][r] = 0;
        }
    }
}

void AStar::kernelDilation(cv::Point pt, cv::Size dilate_kernel)
{
    int dilate_half_y = (dilate_kernel.height - 1) / 2;
    int dilate_half_x = (dilate_kernel.width - 1) / 2;
    int x, y;

    for (int n = -1 * dilate_half_x; n <= dilate_half_x; n++) {
        for (int m = -1 * dilate_half_y; m <= dilate_half_y; m++) {
            x = pt.x + n;
            y = pt.y + m;
            if (x < 0 || x >= MAP_WIDTH || y < 0 || y >= MAP_HEIGHT)
                continue;
            map[x][y] = 1;
        }
    }
}
