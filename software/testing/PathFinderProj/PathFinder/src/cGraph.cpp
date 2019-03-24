#include "../include/cGraph.h"
#include <fstream>


void cGraph::add_vertex(std::string name, const unordered_map<std::string, int>& edges)
{
    // Insert the connected nodes in unordered map
    vertices.insert(unordered_map<std::string, const unordered_map<std::string, int>>::value_type(name, edges));
}

vector<std::string> cGraph::shortest_path(std::string start, std::string finish)
{
    // Second arguments -> distances
    // Find the smallest distance in the already in closed list and push it in -> previous
    unordered_map<std::string, int> distances;
    unordered_map<std::string, std::string> previous;
    vector<std::string> nodes; // Open list
    vector<std::string> path; // Closed list

    auto comparator = [&] (std::string left, std::string right) { return distances[left] > distances[right]; };

    for (auto& vertex : vertices)
    {
        if (vertex.first == start)
        {
            distances[vertex.first] = 0;
        }
        else
        {
            distances[vertex.first] = numeric_limits<int>::max();
        }

        nodes.push_back(vertex.first);
        push_heap(begin(nodes), end(nodes), comparator);
    }

    while (!nodes.empty())
    {
        pop_heap(begin(nodes), end(nodes), comparator);
        std::string smallest = nodes.back();
        nodes.pop_back();

//        std::cout << "Open list: ";
//        for( std::vector<std::string>::const_iterator i = nodes.begin(); i != nodes.end(); ++i)
//            std::cout << *i << ' ';
//        std::cout << std::endl;

        if (smallest == finish)
        {
            while (previous.find(smallest) != end(previous))
            {
                path.push_back(smallest);
                smallest = previous[smallest];
//                std::cout << "Closed list: ";
//                for( std::vector<std::string>::const_iterator i = path.begin(); i != path.end(); ++i)
//                    std::cout << *i << ' ';
//                std::cout << std::endl;
            }

            break;
        }

        if (distances[smallest] == numeric_limits<int>::max())
        {
            break;
        }

        for (auto& neighbor : vertices[smallest])
        {
            int alt = distances[smallest] + neighbor.second;
            if (alt < distances[neighbor.first])
            {
                distances[neighbor.first] = alt;
                previous[neighbor.first] = smallest;
                make_heap(begin(nodes), end(nodes), comparator);
            }
        }
    }

    return path;
}


vector<std::string> cGraph::addMap2Graph(map_t &map, int blockSize, std::string ego_pos, Direction direction)
{
    int ego_row, ego_col, halfBlock = blockSize / 2;
    std::string token, delimiter = ";", destination;
    vector<std::string> posDestinations;

    token = ego_pos.substr(0, ego_pos.find(delimiter));
    ego_col = std::stoi(token);
    token = ego_pos.erase(0, ego_pos.find(delimiter) + delimiter.length());
    ego_row = std::stoi(token);

    for (int row = halfBlock + 1; row < map.size_x - halfBlock - 1; row+=blockSize)
        for (int column = halfBlock + 1; column < map.size_y - halfBlock - 1; column+=blockSize)
        {
            if(map.array[row][column] > 5 && map.array[row][column] != GRAY_VALUE)
            {
                if (row == ego_row && column == ego_col)
                    destination = addPoint2Graph(map.array, row, column, map.size_x, map.size_y, blockSize, direction);
                else
                    destination = addPoint2Graph(map.array, row, column, map.size_x, map.size_y, blockSize, NONE);

                if(destination.compare("") != 0)
                    posDestinations.push_back(destination);
            }
        }
    return posDestinations;
}

// ############################# CAST ARRAY TO DIJKSTRA GRAPH ##################################

std::string cGraph::addPoint2Graph(int **&orig_map, int row, int column, int map_size_x, int map_size_y, int blockSize, Direction egoDirection)
{
    std::string egoNodeNr = std::to_string(row) + ";" + std::to_string(column);
    std::string posdestination = "";

    std::string northNodeNr;
    int northNodeValue = 0;

    std::string westNodeNr;
    int westNodeValue = 0;

    std::string southNodeNr;
    int southNodeValue = 0;

    std::string eastNodeNr;
    int eastNodeValue = 0;


    unordered_map<std::string, int> umap;
//    std::cout<<"egoNodeNr: " << egoNodeNr;

    // just add neighbours  if they are:
    // 1) inside the array
    // 2) not a neighbour of ego node
    // 3) neighbour of ego node and point so same directon
    if(row - blockSize >= 0 && (egoDirection == NONE || egoDirection == NORTH))
    {
        northNodeNr = std::to_string(row-blockSize) + ";" + std::to_string(column);
        northNodeValue = orig_map[row-blockSize][column];

        if (northNodeValue == GRAY_VALUE )
        {
            std::cout << "possible destination point: " << egoNodeNr << endl;
            posdestination = egoNodeNr;
        }
        else if (northNodeValue > 5) // northNodeNr != GRAY_VALUE
        {
            umap.insert({northNodeNr, 255-northNodeValue});
//            std::cout<< " child: " << northNodeNr ;
        }
    }
    if(column - blockSize >= 0 && (egoDirection == NONE || egoDirection == WEST))
    {
        westNodeNr = std::to_string(row) + ";" + std::to_string(column-blockSize);
        westNodeValue = orig_map[row][column-blockSize];
        if (westNodeValue == GRAY_VALUE )
        {
            std::cout << "possible destination point: " << egoNodeNr << endl;
            posdestination = egoNodeNr;
        }
        else if (westNodeValue != GRAY_VALUE && westNodeValue > 5)
        {
            umap.insert({westNodeNr, 255-westNodeValue});
//            std::cout<<" child: " << westNodeNr;
        }
    }

    if(row + blockSize < map_size_y && (egoDirection == NONE || egoDirection == SOUTH))
    {
        southNodeNr = std::to_string(row+blockSize) + ";" + std::to_string(column);
        southNodeValue = orig_map[row+blockSize][column];
        if (southNodeValue == GRAY_VALUE )
        {
            std::cout << "possible destination point: " << egoNodeNr << endl;
            posdestination = egoNodeNr;
        }
        else if (southNodeValue != GRAY_VALUE && southNodeValue > 5)
        {
            umap.insert({southNodeNr, 255-southNodeValue});
//            std::cout<< " child: " << southNodeNr ;
        }

    }

    if(column + blockSize < map_size_x && (egoDirection == NONE || egoDirection == EAST))
    {
        eastNodeNr = std::to_string(row) + ";" + std::to_string(column + blockSize);
        eastNodeValue = orig_map[row][column+blockSize];
        if (eastNodeValue == GRAY_VALUE )
        {
            std::cout << "possible destination point: " << egoNodeNr << endl;
            posdestination = egoNodeNr;
        }
        else if (eastNodeValue != GRAY_VALUE && eastNodeValue > 5)
        {
            umap.insert({eastNodeNr, 255-eastNodeValue});
//            std::cout<<" child: " << eastNodeNr;
        }

    }

    this->add_vertex(egoNodeNr, {umap});

    return posdestination;

    //this->add_vertex(egoNodeNr, {{northNodeNr, northNodeValue}, {westNodeNr, westNodeValue}, {southNodeNr, southNodeValue}, {eastNodeNr,eastNodeValue}});

//    std::cout << endl;
}


