#include "cGraph.h"
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


void cGraph::addMap2Graph(int **&orig_map, int map_size_x, int map_size_y, int blockSize)
{
    int halfBlock = blockSize / 2;
    for (int row = halfBlock + 1; row < map_size_x - halfBlock - 1; row+=blockSize)
        for (int column = halfBlock + 1; column < map_size_y - halfBlock - 1; column+=blockSize)
        {
            if(orig_map[row][column] > 5 && orig_map[row][column] != 127)  //!!!
            {
                addPoint2Graph(orig_map, row, column, map_size_x, map_size_y, blockSize);
            }
        }
}


void cGraph::addPoint2Graph(int **&orig_map, int row, int column, int map_size_x, int map_size_y, int blockSize)
{
    std::string egoNodeNr = std::to_string(row) + ";" + std::to_string(column);

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

    if(row - blockSize >= 0)
    {
        northNodeNr = std::to_string(row-blockSize) + ";" + std::to_string(column);
        northNodeValue = orig_map[row-blockSize][column];

        if (northNodeValue == 127 )
        {
            std::cout << "possible destination point: " << egoNodeNr << endl;
        }
        else if (northNodeValue > 5) // northNodeNr != 127
        {
            umap.insert({northNodeNr, 255-northNodeValue});
//            std::cout<< " child: " << northNodeNr ;
        }
    }
    if(column - blockSize >= 0)
    {
        westNodeNr = std::to_string(row) + ";" + std::to_string(column-blockSize);
        westNodeValue = orig_map[row][column-blockSize];
        if (westNodeValue == 127 )
        {
            std::cout << "possible destination point: " << egoNodeNr << endl;
        }
        else if (westNodeValue != 127 && westNodeValue > 5)
        {
            umap.insert({westNodeNr, 255-westNodeValue});
//            std::cout<<" child: " << westNodeNr;
        }
    }

    if(row + blockSize < map_size_y)
    {
        southNodeNr = std::to_string(row+blockSize) + ";" + std::to_string(column);
        southNodeValue = orig_map[row+blockSize][column];
        if (southNodeValue == 127 )
        {
            std::cout << "possible destination point: " << egoNodeNr << endl;
        }
        else if (southNodeValue != 127 && southNodeValue > 5)
        {
            umap.insert({southNodeNr, 255-southNodeValue});
//            std::cout<< " child: " << southNodeNr ;
        }

    }

    if(column + blockSize < map_size_x)
    {
        eastNodeNr = std::to_string(row) + ";" + std::to_string(column + blockSize);
        eastNodeValue = orig_map[row][column+blockSize];
        if (eastNodeValue == 127 )
        {
            std::cout << "possible destination point: " << egoNodeNr << endl;
        }
        else if (eastNodeValue != 127 && eastNodeValue > 5)
        {
            umap.insert({eastNodeNr, 255-eastNodeValue});
//            std::cout<<" child: " << eastNodeNr;
        }

    }

    this->add_vertex(egoNodeNr, {umap});

    //this->add_vertex(egoNodeNr, {{northNodeNr, northNodeValue}, {westNodeNr, westNodeValue}, {southNodeNr, southNodeValue}, {eastNodeNr,eastNodeValue}});

//    std::cout << endl;
}

void cGraph::addPGM(std::string const&path)
{
    int **orig_map;


    fillMap(path, orig_map);

    transferMapToGraph(orig_map);




    // free disk
    for(int i = 0; i < size_x; ++i)
        delete [] orig_map[i];
    delete [] orig_map;

}

void cGraph::fillMap(std::string const&path, int **&orig_map)
{
    ifstream f_lidarOutput(path);

    std::string mapLine;
    std::string mapSizeToken;

    // get map size from file
    std::getline(f_lidarOutput, mapLine);  // P2 -> not needed
    std::getline(f_lidarOutput, mapLine);  // x y

    mapSizeToken = mapLine.substr(0, mapLine.find(" "));
    this->size_x = std::atoi(mapSizeToken.c_str());

    mapLine = mapLine.substr(mapLine.find(" ")+1, mapLine.length());
    this->size_y = std::atoi(mapLine.substr(0, mapLine.find(" ")).c_str());

    std::cout<<"Map size: \n x: "<<size_x << " y: " << size_y << endl;


    // create array of pointers of pointers
    orig_map = new int*[size_x];
    for(int i = 0; i < size_x; ++i)
        orig_map[i] = new int[size_y];


    // fill array with data points
    int row = 0;
    for (std::string line; std::getline(f_lidarOutput, line); )
    {
        for(int column = 0; column < size_x; column++)
        {
            std::string token = line.substr(0, line.find(" "));
            line = line.substr(line.find(" ")+1, line.length());
            orig_map[row][column] = std::atoi(token.c_str());
        }
        row++;
    }

    f_lidarOutput.close();

}

void cGraph::transferMapToGraph(int **&orig_map)
{

    for(int row = 0; row < size_y; row++)
    {
        for(int column = 0; column < size_x; column++)
        {
            if(orig_map[row][column] != 127)
            {
                this->nodes.push_back(cNode(row, column, orig_map[row][column]));
                //this->nodes.back().addNeighbors();

            }
        }
    }
}


